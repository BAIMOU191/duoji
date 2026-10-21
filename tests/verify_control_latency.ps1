param(
    [string]$HostGcc = 'D:\SoftWare\mingw64\bin\gcc.exe',
    [string]$OutputDir = 'backups/control_latency_20261013'
)
$ErrorActionPreference = 'Stop'
$taskWorkspace = Split-Path $PSScriptRoot -Parent
Push-Location -LiteralPath $taskWorkspace
try {
    New-Item -ItemType Directory -Force -Path $OutputDir | Out-Null
    $taskIncludes = @('-Itests/stubs')
    foreach ($taskDir in @('Application','Common','Drivers','System','SRC/Debug')) {
        $taskIncludes += '-ICH32V00X_APP_ADC_30kg/' + $taskDir
    }
    $taskExe = Join-Path $OutputDir 'control_latency_sim.exe'
    & $HostGcc -std=gnu99 -O2 -Wall -Wextra @taskIncludes tests/control_latency_sim.c -lm -o $taskExe
    if ($LASTEXITCODE -ne 0) { throw 'Current 30kg host simulation compile failed' }
    & $taskExe (Join-Path $OutputDir 'trace.csv') | Tee-Object -FilePath (Join-Path $OutputDir 'results.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Simulation assertions failed' }
    $taskSources = @('Application/A_Config.c','Application/A_Config.h','Application/A_Servo.c','Application/A_UartCmd.c','Common/C_Traj_Planner.c','Common/C_Pos_Ctrl.c','Common/C_Speed_Observer.c','Common/C_Motor_Model.c','Common/C_Task_Scheduler.c')
    $taskSources | ForEach-Object { Get-FileHash -Algorithm SHA256 -LiteralPath (Join-Path 'CH32V00X_APP_ADC_30kg' $_) } |
        Select-Object Path, Hash | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $OutputDir 'source_hashes.json')
    python -B tests/plot_control_latency.py $OutputDir
    if ($LASTEXITCODE -ne 0) { throw 'Plot rendering failed' }
} finally { Pop-Location }
