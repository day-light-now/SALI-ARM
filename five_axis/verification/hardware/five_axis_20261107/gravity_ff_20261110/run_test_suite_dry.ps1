param()
$ErrorActionPreference = "Stop"
$hereWin = $PSScriptRoot
$hereWsl = "/mnt/" + $hereWin.Substring(0, 1).ToLower() + "/" + $hereWin.Substring(2).Replace('\', '/')
Write-Host "Offline suite validation (no driver, no motion):"
wsl.exe -d Ubuntu-24.04 --exec /bin/bash -c "cd '$hereWsl' && bash run_test_suite.sh --dry"
if ($LASTEXITCODE -ne 0) { Write-Host "SUITE DRY CHECK FAILED" -ForegroundColor Red; exit $LASTEXITCODE }
Write-Host "Suite plans are ready for the next hardware session." -ForegroundColor Green
