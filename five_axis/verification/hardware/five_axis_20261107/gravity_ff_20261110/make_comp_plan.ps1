param(
  [Parameter(Mandatory = $true)][string]$Plan
)
$ErrorActionPreference = "Stop"
$hereWin = $PSScriptRoot
$hereWsl = "/mnt/" + $hereWin.Substring(0, 1).ToLower() + "/" + $hereWin.Substring(2).Replace('\', '/')
$resolved = (Resolve-Path -LiteralPath $Plan).Path
$drive = $resolved.Substring(0, 1).ToLower()
$wslPlan = "/mnt/$drive/" + $resolved.Substring(2).Replace('\', '/')
$name = [IO.Path]::GetFileNameWithoutExtension($resolved) + "_comp"
Write-Host "Input : $resolved"
Write-Host "Output: $hereWsl/plans/${name}_20261110.json"
wsl.exe -d Ubuntu-24.04 --exec /bin/bash -c "cd '$hereWsl' && python3 compensate_plan.py '$wslPlan' '$name'"
if ($LASTEXITCODE -ne 0) {
  Write-Host "Conversion rejected or failed (see message above); no plan was written." -ForegroundColor Yellow
  exit $LASTEXITCODE
}
Write-Host "Model-compensated plan written under the gravity_ff_20261110/plans directory." -ForegroundColor Green
