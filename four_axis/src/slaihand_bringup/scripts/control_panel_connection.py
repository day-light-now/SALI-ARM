"""Bounded local driver lifecycle, no motor services or serial control writes.

Only the exact four-axis CANUSB launch in this ROS domain may be stopped.
Port ownership and process exit are checked before a replacement is started.
"""
import os
from pathlib import Path
import re
import signal
import subprocess
import time

PORT = '/dev/serial/by-id/usb-HDSC_CDC_Device_00000000050C-if00'
USBIPD = '/mnt/c/Program Files/usbipd-win/usbipd.exe'


def launches(port, domain):
    result=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            argv=(path/'cmdline').read_bytes().decode().rstrip('\0').split('\0')
            environment=(path/'environ').read_bytes().split(b'\0')
            at=argv.index('launch')
            if argv[at:at+3]!=['launch','slaihand_bringup','slaihand_canusb_motion.launch.py']:continue
            if 'port:='+port not in argv:continue
            current=next((entry.split(b'=',1)[1].decode() for entry in environment
                          if entry.startswith(b'ROS_DOMAIN_ID=')),'0')
            if current==domain:result.append(int(path.name))
        except (OSError,ValueError,UnicodeError):continue
    return result


def alive(pid):
    try:return Path('/proc/%d/stat'%pid).read_text().split(') ',1)[1].split()[0]!='Z'
    except (OSError,IndexError):return False


class DriverConnection:
    def __init__(self, log_directory, port=PORT):
        self.port=port
        self.domain=os.environ.get('ROS_DOMAIN_ID','0')
        self.log_directory=Path(log_directory)
        self.proc=None

    def owners(self):
        if not Path(self.port).exists():return []
        result=subprocess.run(['fuser',self.port],capture_output=True,text=True,timeout=3)
        return [int(value) for value in result.stdout.split() if value.isdigit()]

    def inspect(self):
        return {'port_present':Path(self.port).exists(),
                'launch_pids':launches(self.port,self.domain),
                'port_owner_pids':self.owners()}

    def restart(self, reset_usb, cancelled, progress):
        self.proc=None  # cancellation may only target a process started by this attempt
        matches=launches(self.port,self.domain)
        if len(matches)>1:raise RuntimeError('检测到多个匹配驱动，拒绝自动处理')
        for pid in matches:
            # Recheck immediately before signalling: never kill by a broad name.
            if pid not in launches(self.port,self.domain):raise RuntimeError('驱动身份变化，请重新检查连接')
            progress('正在停止旧驱动')
            os.kill(pid,signal.SIGINT)
            deadline=time.monotonic()+6
            while alive(pid) and time.monotonic()<deadline:
                if cancelled.wait(.05):raise RuntimeError('连接操作已取消')
            if alive(pid):raise RuntimeError('旧驱动未退出，未启动第二个驱动')
        if self.owners():raise RuntimeError('串口仍被占用，未启动第二个驱动')
        if cancelled.is_set():raise RuntimeError('连接操作已取消')
        if reset_usb:
            if not Path(USBIPD).is_file():raise RuntimeError('未找到USB转发工具，请检查适配器连接')
            listing=subprocess.run([USBIPD,'list'],capture_output=True,text=True,timeout=8)
            devices=re.findall(r'^\s*(\d+-\d+(?:\.\d+)*)\s+2e88:4603\s+(.+)$',listing.stdout,re.M|re.I)
            if len(devices)!=1:raise RuntimeError('未找到唯一的原CANUSB适配器，请重新插好USB')
            busid,state=devices[0]
            progress('正在重新连接USB转发')
            if 'Attached' in state:
                subprocess.run([USBIPD,'detach','--busid',busid],check=True,
                               capture_output=True,text=True,timeout=8)
            if cancelled.is_set():raise RuntimeError('连接操作已取消')
            attached=subprocess.run([USBIPD,'attach','--wsl','--busid',busid],
                                    capture_output=True,text=True,timeout=15)
            if attached.returncode:raise RuntimeError('USB转发失败，请检查Windows适配器状态')
        deadline=time.monotonic()+4
        while not Path(self.port).exists() and time.monotonic()<deadline:
            if cancelled.wait(.1):raise RuntimeError('连接操作已取消')
        if not Path(self.port).exists():raise RuntimeError('WSL未发现CANUSB串口，请检查USB连接与转发')
        if self.owners() or launches(self.port,self.domain):raise RuntimeError('发现已有串口持有者，拒绝重复启动')
        if cancelled.is_set():raise RuntimeError('连接操作已取消')
        self.log_directory.mkdir(parents=True,exist_ok=True)
        log=self.log_directory/('driver_%s_%d.log'%(time.strftime('%Y%m%d_%H%M%S'),time.time_ns()%1000000))
        progress('正在启动未使能驱动，等待四轴反馈')
        with log.open('x') as stream:
            self.proc=subprocess.Popen(['/opt/ros/jazzy/bin/ros2','launch','slaihand_bringup',
                'slaihand_canusb_motion.launch.py','port:='+self.port],
                stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        return self.proc,log

    def cancel_owned_start(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
