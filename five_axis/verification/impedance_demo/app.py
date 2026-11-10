"""Standalone native MuJoCo viewer + Tk panel. No ROS/CAN/serial/network."""
import time
import tkinter as tk
from tkinter import ttk
import numpy as np
from model import Simulation, mujoco
import mujoco.viewer


def main():
    sim = Simulation()
    root = tk.Tk()
    root.title('SLAI 控制实验室 · MuJoCo · 仅仿真')
    root.geometry('650x690')
    ttk.Style(root).configure('TButton', font=('Microsoft YaHei', 11), padding=7)
    ttk.Label(root, text='肩部四轴 · 重力补偿 + 关节阻抗', font=('Microsoft YaHei',17,'bold')).pack(pady=12)
    ttk.Label(root, text='暂定质量/惯量；无碰撞；无实机输出。独立于 RViz / 8767。').pack()
    mode_text = tk.StringVar()
    ttk.Label(root, textvariable=mode_text, font=('Microsoft YaHei',12)).pack(pady=10)
    automatic = [False]
    timeline = [0.]
    phase = [-1]
    def mode(name):
        automatic[0] = False
        sim.set_mode(name)
    frame = ttk.Frame(root); frame.pack(fill='x', padx=15)
    for title, name in [('① 无补偿：观察下垂','off'), ('② 补偿：推后停在新位置','float'), ('③ 弹簧：推后回到目标','spring')]:
        ttk.Button(frame, text=title, command=lambda n=name: mode(n)).pack(fill='x', pady=3)
    controls = ttk.Frame(root); controls.pack(pady=10)
    ttk.Button(controls, text='↑ 向上推 8 N', command=lambda: sim.push((0,0,8))).grid(row=0,column=0,padx=4)
    ttk.Button(controls, text='↓ 向下推 8 N', command=lambda: sim.push((0,0,-8))).grid(row=0,column=1,padx=4)
    ttk.Button(controls, text='当前姿态设为目标', command=sim.capture).grid(row=1,column=0,pady=5)
    def reset():
        automatic[0] = False
        sim.reset()
        sim.set_mode('spring')
    ttk.Button(controls, text='重置到零位', command=reset).grid(row=1,column=1,pady=5)
    def demo_pose():
        automatic[0] = False
        sim.enter_demo()
        sim.set_mode('spring')
    ttk.Button(controls, text='进入演示姿态 [25°,65°,0°,-25°]', command=demo_pose).grid(row=2,column=0,columnspan=2,pady=5)
    def auto():
        automatic[0] = True
        timeline[0] = time.monotonic()
        phase[0] = -1
    ttk.Button(root, text='▶ 自动表演：三种控制对比（约 18 秒）', command=auto).pack(fill='x',padx=18)
    feedback = tk.StringVar()
    ttk.Label(root, textvariable=feedback, font=('Consolas',12), justify='left').pack(pady=10)
    ttk.Label(root, text=chr(10).join([
        '补偿模式带阻尼，不是位置锁定；弹簧模式包含重力前馈。',
        '↑/↓按钮在 link4 重心施加 0.4 秒世界 Z 方向外力。',
        '启动和重置均为 [0°,0°,0°,0°]；演示姿态需单独进入。',
        'MuJoCo 窗口可用鼠标旋转/缩放。']), justify='left').pack(pady=8)
    status = tk.StringVar(value='当前为零位；自动表演会切到演示姿态，结束后可重置到零位。')
    ttk.Label(root,textvariable=status,wraplength=600).pack()
    view = mujoco.viewer.launch_passive(sim.model, sim.data, show_left_ui=False, show_right_ui=False)
    view.cam.lookat[:] = [0,.12,-.05]
    view.cam.distance = 1.1
    view.cam.azimuth = 130
    view.cam.elevation = -15
    last = [time.monotonic()]
    def tick():
        if not view.is_running():
            root.destroy(); return
        now = time.monotonic()
        if automatic[0]:
            elapsed = now-timeline[0]
            stages = [(0,'off',False), (3,'float',False), (5,'float',True),
                      (9,'spring',False), (11,'spring',True), (18,'spring',False)]
            index = max(i for i,s in enumerate(stages) if elapsed>=s[0])
            if index != phase[0]:
                phase[0] = index
                _, name, push = stages[index]
                if index in [0,1,3]: sim.enter_demo()
                sim.set_mode(name)
                if push: sim.push((0,0,8))
                status.set(['① 无补偿：重力使手臂下垂','② 重力前馈 + 阻尼：保持附近姿态',
                            '② 施加外力：让开，撤力后不回原目标',
                            '③ 重力前馈 + 弹簧阻尼：保持目标',
                            '③ 施加外力：让开，撤力后柔和回位',
                            '演示结束；可以继续手动操作'][index])
                if index == 5: automatic[0] = False
        count = max(1,min(50,int((now-last[0])/sim.model.opt.timestep)))
        last[0] += count*sim.model.opt.timestep
        if now-last[0]>.2: last[0]=now
        try:
            with view.lock(): sim.step(count)
            view.sync()
        except Exception as error:
            status.set('仿真停止：'+str(error)); view.close(); return
        mode_text.set({'off':'无重力补偿（保留阻尼）','float':'重力补偿 + 阻尼（无位置弹簧）',
                       'spring':'重力前馈 + 关节弹簧阻尼'}[sim.mode])
        feedback.set('       反馈角度 °    目标角度 °    控制力矩 N·m'+chr(10)+chr(10).join(
            f'J{i+1}    {q:8.2f}       {r:8.2f}       {t:8.3f}'
            for i,(q,r,t) in enumerate(zip(np.rad2deg(sim.data.qpos),np.rad2deg(sim.target),sim.torque))))
        root.after(15,tick)
    def close():
        view.close(); root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    root.after(15,tick)
    try: root.mainloop()
    finally: view.close()


if __name__ == '__main__': main()
