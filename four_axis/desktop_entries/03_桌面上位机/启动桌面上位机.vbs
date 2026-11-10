Option Explicit
Dim shell
Set shell = CreateObject("WScript.Shell")
shell.CurrentDirectory = "D:\CodexData\Projects\" & ChrW(&H673A) & ChrW(&H68B0) & ChrW(&H81C2) & "\slaihand_ros2_ws"
shell.Run """C:\Users\ASUS\AppData\Local\Programs\Python\Python312\pythonw.exe"" -m slai_console.launch", 1, False
