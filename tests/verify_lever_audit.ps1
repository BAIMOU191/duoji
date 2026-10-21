param([string]$HostGcc = 'D:\SoftWare\mingw64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$taskWorkspace = Split-Path $PSScriptRoot -Parent
Push-Location -LiteralPath $taskWorkspace
try {
    $taskOutput = 'backups/lever_audit_20261013'
    New-Item -ItemType Directory -Force $taskOutput | Out-Null
    $taskIncludes = @('-Itests/stubs')
    foreach ($taskDir in @('Application','Common','Drivers','System','SRC/Debug')) { $taskIncludes += '-ICH32V00X_APP_ADC_30kg/' + $taskDir }
    foreach ($taskTest in @('lever_hold_sim','control_chain_recheck','uart_driver_sim')) {
        $taskExe = Join-Path $taskOutput ($taskTest + '.exe')
        & $HostGcc -std=gnu99 -O2 -Wall -Wextra -Werror @taskIncludes ('tests/' + $taskTest + '.c') -lm -o $taskExe
        if ($LASTEXITCODE -ne 0) { throw "$taskTest compile failed" }
        if ($taskTest -eq 'lever_hold_sim') {
            & $taskExe (Join-Path $taskOutput 'summary_original.csv') (Join-Path $taskOutput 'trace_original.csv') | Tee-Object (Join-Path $taskOutput 'disturb_original.txt')
            if ($LASTEXITCODE -ne 0) { throw 'Lever simulation failed' }
            & $taskExe (Join-Path $taskOutput 'sweep_original.csv')
        } else { & $taskExe | Tee-Object (Join-Path $taskOutput ($taskTest + '.txt')) }
        if ($LASTEXITCODE -ne 0) { throw "$taskTest failed" }
    }
    & $HostGcc -std=gnu99 -O2 -Wall -Wextra -Werror -DAUDIT_REAL_PROTECT @taskIncludes tests/lever_hold_sim.c -lm -o (Join-Path $taskOutput 'lever_protected.exe')
    if ($LASTEXITCODE -ne 0) { throw 'Protected lever simulation compile failed' }
    & (Join-Path $taskOutput 'lever_protected.exe') (Join-Path $taskOutput 'summary_protected.csv') (Join-Path $taskOutput 'trace_protected.csv') | Tee-Object (Join-Path $taskOutput 'disturb_protected.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Protected lever simulation failed' }
    & $HostGcc -std=gnu99 -O0 -Wall -Wextra -Werror @taskIncludes tests/lever_hold_sim.c -lm -o (Join-Path $taskOutput 'lever_O0.exe')
    if ($LASTEXITCODE -ne 0) { throw 'O0 compile failed' }
    & (Join-Path $taskOutput 'lever_O0.exe') (Join-Path $taskOutput 'summary_O0.csv') (Join-Path $taskOutput 'trace_O0.csv') | Set-Content (Join-Path $taskOutput 'disturb_O0.txt')
    if ($LASTEXITCODE -ne 0) { throw 'O0 run failed' }
    if ((Get-FileHash (Join-Path $taskOutput 'trace_original.csv')).Hash -ne (Get-FileHash (Join-Path $taskOutput 'trace_O0.csv')).Hash) { throw 'O0/O2 mismatch' }
    python -B tests/build_hold_experiments.py $HostGcc
    if ($LASTEXITCODE -ne 0) { throw 'Isolated hold experiments failed' }
    & tests/verify_control_latency.ps1 -HostGcc $HostGcc -OutputDir (Join-Path $taskOutput 'latency_recheck')
    if (!$?) { throw 'Previous findings recheck failed' }
    python -B tests/plot_lever_audit.py $taskOutput
    if ($LASTEXITCODE -ne 0) { throw 'Analysis/plot failed' }
    'PASS: old findings reproduced, current driver/protection checked, lever variants simulated; production code untouched'
} finally { Pop-Location }
