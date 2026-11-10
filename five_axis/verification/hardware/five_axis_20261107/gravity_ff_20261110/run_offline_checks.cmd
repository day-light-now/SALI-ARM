@echo off
setlocal
set "HERE=%~dp0"
wsl.exe -d Ubuntu-24.04 --exec /bin/bash -c "cd \"$(wslpath -a '%HERE%')\" && bash run_offline_checks.sh"
pause
