"""Separate operator-controlled J1-J4 holding calibration, no automatic motion."""
import json
import queue
import threading
import time
import uuid

from .core import ROOT
from .native_mit_console import NativeTrialRequest, run_native_trial, Reader, load_mapping
from .live_trial import fill_preflight
from .watchdog_check import make_event_writer
from .load_calibration import save_record,inventory,ARCHIVE


def main():
    import tkinter as tk
    from tkinter import ttk
    root=tk.Tk();root.title('SLAI · 实机负载补偿标定 · 独立工具');root.geometry('1140x820')
    ui=ttk.Frame(root,padding=16);ui.pack(fill='both',expand=True)
    ttk.Label(ui,text='实机负载补偿标定 · J1–J4逐轴建档',font=('Microsoft YaHei',17,'bold')).pack(anchor='w')
    ttk.Label(ui,wraplength=1090,text='不读取仿真重量。每次仅所选一轴在当前位置保持30秒，不自动变姿态、不回零、不写电机参数。\n'
        'J1估算门槛8Nm、J2为10Nm、J3/J4为2Nm；不是驱动硬限幅。J5不查询、不使能。\n'
        '记录的是当前姿态的驱动请求估计，含摩擦等影响；一个点不等于全行程标定完成，保存后也不会自动用于控制。').pack(anchor='w',pady=8)
    bar=ttk.Frame(ui);bar.pack(fill='x')
    port=tk.StringVar(value='COM9');ttk.Label(bar,text='串口').pack(side='left')
    ttk.Entry(bar,textvariable=port,width=12).pack(side='left')
    joint=tk.IntVar(value=2);radios=[]
    for i in range(1,5):
        b=ttk.Radiobutton(bar,text=f'J{i}',value=i,variable=joint);b.pack(side='left',padx=12);radios.append(b)
    form=ttk.Frame(ui);form.pack(fill='x',pady=8)
    pose=tk.StringVar(value='姿态1');load=tk.StringVar(value='无末端额外负载');approach=tk.StringVar(value='未记录')
    ttk.Label(form,text='姿态标签').pack(side='left');ttk.Entry(form,textvariable=pose,width=14).pack(side='left')
    ttk.Label(form,text='负载说明').pack(side='left',padx=6);ttk.Entry(form,textvariable=load,width=25).pack(side='left')
    ttk.Label(form,text='到该姿态的方向').pack(side='left',padx=6)
    ttk.Combobox(form,textvariable=approach,values=('模型正向','模型反向','未记录'),state='readonly',width=10).pack(side='left')
    support=tk.BooleanVar();physical=tk.BooleanVar();output=tk.BooleanVar()
    for v,text in ((support,'机械臂固定、有可靠防坠支撑和局部净空，可立即切电；支撑不会卡住转轴'),
        (physical,'已记录实际负载和接近方向；采样期间外部支撑不分担待标关节的静态负载（不能确认则不采样）'),
        (output,'接受所选单轴30秒当前位置接管与有界积分候选：前馈≤2Nm，变化≤0.5Nm/s；驱动连续限流未核实')):
        ttk.Checkbutton(ui,text=text,variable=v).pack(anchor='w',pady=3)
    status=tk.StringVar();ttk.Label(ui,textvariable=status,wraplength=1090).pack(anchor='w',pady=6)
    latched=[False]
    def refresh():
        if latched[0]:status.set('输出状态未知，入口已锁定；保持支撑，现场切电检查。');return
        counts=inventory();status.set('已记录合格姿态点：'+'；'.join(f'J{i}={len(counts[i])}' for i in range(1,5))+'。数量不代表全行程覆盖。')
    refresh()
    events=queue.Queue();cancel=threading.Event();busy=[False];closing=[False]
    def worker(mode,settings):
        reader=None
        try:
            reader=Reader(settings['port'],load_mapping());reader.serial.write_timeout=.025
            if mode=='readonly':
                fill_preflight(reader,cancel)
                events.put('；'.join(f'J{i} 模式{reader.state.regs[i].get(10)} / TIMEOUT{reader.state.regs[i].get(9)} / 状态{reader.state.feedback.get(i,{}).get("state_code")}' for i in range(1,5)))
                return
            folder=ROOT/'tmp/load_calibration';folder.mkdir(parents=True,exist_ok=True)
            rid=uuid.uuid4().hex;path=folder/(time.strftime('%Y%m%d_%H%M%S')+'_'+rid[:8]+'.jsonl')
            with path.open('x',encoding='utf-8') as handle:
                audit=make_event_writer(handle,lambda row:None)
                events.put(f'J{settings["joint"]}当前位置保持30秒开始；不自动变姿态。')
                result=run_native_trial(reader,NativeTrialRequest(True,True,True,rid,
                    'calibration_hold',joint=settings['joint'],holding_bias_acknowledged=True),
                    audit,cancel,connection_id=rid)
            trial_path=path.with_suffix('.json')
            trial_path.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
            events.put(json.dumps(result,ensure_ascii=False));events.put('结果：'+str(trial_path))
            if result.get('calibration_record',{}).get('accepted'):
                r=save_record(result,trial_path,pose_label=settings['pose'],approach=settings['approach'],
                    load_note=settings['load'],conditions_confirmed=True)
                events.put(f'已保存J{r["joint"]}姿态点，模型坐标保持请求估计{r["holding_request_estimate_model_nm"]:.3f}Nm。档案：{ARCHIVE}')
            else:events.put('本点未通过稳定/停止确认判据，未写入合格标定档案。')
            if result['final_controller_state']=='UNKNOWN':
                events.put(('latch',True));events.put('状态未知：保持支撑，现场切电检查；不要重复启动。')
        except Exception as exc:events.put('检查/标定结束：'+str(exc))
        finally:
            if reader:reader.close()
            events.put(None)
    def launch(mode):
        if busy[0] or latched[0]:return
        if mode!='readonly' and not all(v.get() for v in (support,physical,output)):
            status.set('请落实并确认三项现场条件；不确定支撑分担负载时不要开始采样。');return
        if mode!='readonly' and (not pose.get().strip() or not load.get().strip()):
            status.set('请填写姿态标签和负载说明。');return
        settings=dict(port=port.get().strip(),joint=joint.get(),pose=pose.get(),load=load.get(),
            approach={'模型正向':'positive','模型反向':'negative','未记录':'unknown'}[approach.get()])
        busy[0]=True;cancel.clear()
        for b in radios:b.config(state='disabled')
        threading.Thread(target=worker,args=(mode,settings),daemon=True).start()
    buttons=ttk.Frame(ui);buttons.pack(fill='x',pady=8)
    read=ttk.Button(buttons,text='只读检查（不使能）',command=lambda:launch('readonly'));read.pack(side='left')
    start=ttk.Button(buttons,text='所选轴保持30秒并记录',command=lambda:launch('calibration'));start.pack(side='left',padx=10)
    ttk.Button(buttons,text='停止·尝试四轴失能',command=cancel.set).pack(side='left')
    log=tk.Text(ui,height=20,wrap='word');log.pack(fill='both',expand=True)
    def poll():
        try:
            while True:
                value=events.get_nowait()
                if value==('latch',True):latched[0]=True
                elif value is None:busy[0]=False;output.set(False);physical.set(False);refresh()
                else:log.insert('end',value+'\n');log.see('end')
        except queue.Empty:pass
        for b in (read,start,*radios):b.config(state='disabled' if busy[0] or latched[0] else 'normal')
        if closing[0] and not busy[0]:root.destroy();return
        root.after(60,poll)
    def close():closing[0]=True;cancel.set()
    root.protocol('WM_DELETE_WINDOW',close);root.after(60,poll);root.mainloop()


if __name__=='__main__':main()
