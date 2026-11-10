param(
    [Parameter(Mandatory = $true)][string]$PlanPath,
    [switch]$DryRun,
    [switch]$PlanOnly,
    [ValidateSet("hybrid", "model", "iter")][string]$Strategy = "hybrid",
    [string]$Tag = ""
)
$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

if (-not (Test-Path -LiteralPath $PlanPath)) {
    Write-Host "Plan file not found: $PlanPath"
    exit 1
}

try {
    $plan = Get-Content -LiteralPath $PlanPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $count = @($plan.waypoints).Count
} catch {
    Write-Host "Cannot parse plan JSON: $($_.Exception.Message)"
    exit 1
}
Write-Host "Plan: $PlanPath"
Write-Host "Waypoints: $count"

$wslBase = (& wsl.exe -d Ubuntu-24.04 --exec wslpath -u "$scriptDir").Trim()
$wslPlan = (& wsl.exe -d Ubuntu-24.04 --exec wslpath -u "$PlanPath").Trim()
if (-not $wslPlan.StartsWith("/")) {
    Write-Host "WSL path conversion failed: $wslPlan"
    exit 1
}

if (-not $DryRun -and -not $PlanOnly) {
    Write-Host ""
    Write-Host "=== SAFETY CONFIRMATION ==="
    Write-Host "Pipeline: read-only TIMEOUT admission -> artifact -> candidate driver ->"
    Write-Host "strategy '$Strategy' (hybrid = model-compensated run + one measured correction;"
    Write-Host "model = model-compensated run only; iter = legacy baseline+2 iterations) ->"
    Write-Host "comparison/report -> stop driver. Gravity FF stays 0; 4 deg guard / STOP /"
    Write-Host "limits / admission are unchanged."
    $answer = Read-Host "Confirm supported / clearance / power-cut ready. Type RUN to start"
    if ($answer -ne "RUN") {
        Write-Host "Cancelled."
        exit 2
    }
}

if ([string]::IsNullOrWhiteSpace($Tag)) {
    $Tag = "drag_" + (Get-Date -Format "yyyyMMdd_HHmmss")
}
$mode = if ($DryRun) { "dry" } elseif ($PlanOnly) { "plan" } else { "run" }
Write-Host "mode=$mode strategy=$Strategy"

& wsl.exe -d Ubuntu-24.04 --exec /bin/bash "$wslBase/compensate.sh" $wslPlan $Tag $mode $Strategy
$code = $LASTEXITCODE
Write-Host "compensate.sh exit=$code tag=$Tag"
exit $code
