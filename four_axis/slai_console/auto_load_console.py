"""Independent preview-first automatic load acquisition UI; no auto-connect."""
import hashlib
import json
import queue
import threading
import time
import uuid

from .core import ROOT
from .clock import now as clock_now
from .native_mit_console import Reader,load_mapping
from .live_trial import fill_preflight
from .watchdog_check import make_event_writer
from .auto_load_acquisition import build_plan
from .auto_load_runner import run_acquisition
from .load_calibration import save_campaign_points

LIMIT_FILE=ROOT/'verification/hardware/load_calibration/measured_limits_20261102.json'


def write_checkpoint(path,data):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    temporary.replace(path)


def preview_from_reader(reader,joint=1):
    mapping=load_mapping();mapping_file=ROOT/'src/slaihand_bringup/config/hardware_mapping.yaml'
    digest=hashlib.sha256(mapping_file.read_bytes()).hexdigest()
    config=json.loads(LIMIT_FILE.read_text(encoding='utf-8'))
    if config['mapping_sha256']!=digest or reader.state.mapping!=mapping:
        raise ValueError('Zero/sign changed after limit confirmation; re-review physical coordinates')
    settings=config['joints'].get(str(joint))
    if settings is None:raise ValueError('This axis has no measured automatic-scan envelope yet')
    q=[]
    for i in range(1,5):
        if (not reader.state.ready(i) or i not in reader.state.feedback or
            reader.state.feedback[i]['state_code']!=0 or reader.state.regs[i].get(10)!=1 or
            reader.state.regs[i].get(9)!=2000):
            raise ValueError(f'J{i}: fresh MIT/2000/normal-disabled feedback required')
        m=mapping[i];q.append(m['hardware_sign']*(reader.state.feedback[i]['position_rad']-m['zero_offset_rad']))
    return build_plan(joint=joint,initial_pose_rad=q,mechanical_limits_deg=settings['mechanical_limits_deg'],
        mapping_sha256=digest,physical_limit_evidence=config['source'],physical_limits_confirmed=settings['physical_limits_confirmed'],
        scan_limits_deg=settings['scan_limits_deg'],step_deg=10.,speed_rad_s=.02)


def main():
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('SLAI · 自动负载补偿采集 · J1');root.geometry('1140x830')
    frame=ttk.Frame(root,padding=16);frame.pack(fill='both',expand=True)
    ttk.Label(frame,text='自动负载补偿采集 · 先预览，再明确启动',font=('Microsoft YaHei',17,'bold')).pack(anchor='w')
    ttk.Label(frame,wraplength=1090,text='当前首轮：J1软件角度−70°～+70°，目标端点−69.5°～+69.5°；J1 sign=+1，编码器增加为正。\n'
        '只使能J1，其他轴姿态必须保持不变；J5不查询。低速分段到位→稳定记录→反向重复→返回预览起点。\n'
        '保留J1总请求估算8Nm、原生Kp160/Kd5、参考速度≤0.02rad/s；驱动持续硬限流未核实。\n'
        '同时记录四轴姿态/速度/状态/力矩估计；未使能轴和运动段不能算静态重力标签。不使用仿真重量。').pack(anchor='w',pady=10)
    bar=ttk.Frame(frame);bar.pack(fill='x')
    port=tk.StringVar(value='COM9');label=tk.StringVar(value='工作姿态A');load=tk.StringVar(value='无末端额外负载')
    for title,var,width in (('串口',port,10),('工作姿态标签',label,16),('实际负载',load,24)):
        ttk.Label(bar,text=title).pack(side='left',padx=4);ttk.Entry(bar,textvariable=var,width=width).pack(side='left')
    check=[]
    for text in ('机械臂固定且有可靠防坠；可立即现场切电，人员不进入运动范围',
        '已检查J1整段−70°～+70°路径与线缆无干涉；允许本次有限正反向扫描',
        '采样时外部支撑不分担J1负载，J2–J4被可靠保持；不能确认则不启动',
        '接受实机误差有界积分候选（J1前馈≤4Nm、变化≤0.5Nm/s），数据先记录，不自动应用'):
        v=tk.BooleanVar(value=False);check.append(v);ttk.Checkbutton(frame,text=text,variable=v).pack(anchor='w',pady=3)
    status=tk.StringVar(value='未连接；先释放其他软件串口，再点只读预览。')
    ttk.Label(frame,textvariable=status,wraplength=1090).pack(anchor='w',pady=8)
    buttons=ttk.Frame(frame);buttons.pack(fill='x',pady=6)
    log=tk.Text(frame,height=22,wrap='word');log.pack(fill='both',expand=True)
    events=queue.Queue();cancel=threading.Event();busy=[False];latched=[False];closing=[False];reviewed=[None]
    def worker(mode,settings):
        reader=None
        try:
            reader=Reader(settings['port'],load_mapping());reader.serial.write_timeout=.025
            fill_preflight(reader,cancel)
            if cancel.is_set():raise ValueError('Cancelled before motion')
            if mode=='preview':
                plan=preview_from_reader(reader)
                folder=ROOT/'tmp/auto_load_preview';folder.mkdir(parents=True,exist_ok=True)
                path=folder/(uuid.uuid4().hex[:12]+'.json')
                path.write_text(json.dumps(dict(plan=plan.serializable(),plan_sha256=plan.fingerprint),
                    ensure_ascii=False,indent=2),encoding='utf-8')
                events.put(('plan',plan));events.put(('log','只读预览已保存：'+str(path)))
                return
            plan=settings['plan']
            folder=ROOT/'tmp/auto_load_acquisition';folder.mkdir(parents=True,exist_ok=True)
            path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.jsonl')
            with path.open('x',encoding='utf-8') as stream:
                result=run_acquisition(reader,plan,make_event_writer(stream,lambda row:None),cancel,
                    acknowledged=True,progress=lambda text:events.put(('log',text)),
                    checkpoint_callback=lambda data:write_checkpoint(path.with_suffix('.checkpoint.json'),data))
            result_path=path.with_suffix('.json')
            result_path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            events.put(('log',json.dumps(result,ensure_ascii=False)));events.put(('log','结果：'+str(result_path)))
            if result['fresh_stop_confirmation']:
                records=save_campaign_points(result,result_path,pose_label=settings['label'],
                    load_note=settings['load'],conditions_confirmed=True)
                events.put(('log',f'已归档{len(records)}个合格点。整条路线完成={result["campaign_complete"]}；尚未完成全姿态重力辨识。'))
            if result['final_controller_state']=='UNKNOWN':events.put(('latch',True))
            events.put(('plan',None))
        except Exception as exc:events.put(('log','预览/采集结束：'+str(exc)));events.put(('plan',None))
        finally:
            if reader:reader.close()
            events.put(('done',None))
    def launch(mode):
        if busy[0] or latched[0]:return
        if mode=='run' and (reviewed[0] is None or not all(v.get() for v in check)):
            status.set('先完成只读预览，并逐项落实四项现场条件。');return
        if mode=='run' and (not label.get().strip() or not load.get().strip()):
            status.set('请填写工作姿态标签和实际负载。');return
        settings=dict(port=port.get().strip(),plan=reviewed[0],label=label.get(),load=load.get())
        busy[0]=True;cancel.clear()
        threading.Thread(target=worker,args=(mode,settings),daemon=True).start()
    preview=ttk.Button(buttons,text='只读预览路线（不使能）',command=lambda:launch('preview'));preview.pack(side='left')
    start=ttk.Button(buttons,text='明确启动本次J1自动采集',command=lambda:launch('run'),state='disabled');start.pack(side='left',padx=10)
    ttk.Button(buttons,text='停止·尝试四轴失能',command=cancel.set).pack(side='left')
    def poll():
        try:
            while True:
                kind,value=events.get_nowait()
                if kind=='log':log.insert('end',value+'\n');log.see('end')
                elif kind=='plan':
                    reviewed[0]=value
                    if value:
                        count=sum(p.collect for p in value.points)
                        status.set(f'预览：{len(value.points)}段、最多{count}个姿态样本；完整预算{value.total_budget_s/60:.1f}分钟。实际稳定后可提前结束各采样等待。')
                elif kind=='latch':latched[0]=True;status.set('输出状态未知：已锁定。保持支撑，现场切电检查，不重复启动。')
                elif kind=='done':busy[0]=False;[v.set(False) for v in check]
        except queue.Empty:pass
        preview.config(state='disabled' if busy[0] or latched[0] else 'normal')
        start.config(state='normal' if reviewed[0] is not None and not busy[0] and not latched[0] else 'disabled')
        if closing[0] and not busy[0]:root.destroy();return
        root.after(60,poll)
    def close():closing[0]=True;cancel.set()
    root.protocol('WM_DELETE_WINDOW',close);root.after(60,poll);root.mainloop()


if __name__=='__main__':main()
