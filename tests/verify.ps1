param(
    [string]$HostGcc = 'D:\SoftWare\mingw64\bin\gcc.exe',
    [string]$WchBin = 'D:\SoftWare\MounRiver\MounRiver_Studio2\resources\app\resources\win32\components\WCH\Toolchain\RISC-V Embedded GCC12\bin',
    [string]$Make = 'D:\SoftWare\MounRiver\MounRiver_Studio2\resources\app\resources\win32\others\Build_Tools\Make\bin\make.exe'
)
$ErrorActionPreference = 'Stop'
$taskWorkspace = Split-Path $PSScriptRoot -Parent
$taskProject = Join-Path $taskWorkspace 'CH32V00X_APP_ADC_30kg'
$taskOutput = Join-Path $taskWorkspace 'backups/calibration_review_20261013'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
Set-Location -LiteralPath $taskWorkspace
$taskIncludes = @('-Itests/stubs')
foreach ($taskDir in @('Application','Common','Drivers','System','SRC/Debug')) {
    $taskIncludes += '-I' + (Join-Path $taskProject $taskDir)
}
foreach ($taskTest in @('runtime_parameters','calibration_firmware_sim')) {
    & $HostGcc -std=gnu99 -O2 -Wall -Wextra @taskIncludes ('tests/' + $taskTest + '.c') -lm -o (Join-Path $taskOutput ($taskTest + '.exe'))
    if ($LASTEXITCODE -ne 0) { throw "$taskTest compile failed" }
}
& (Join-Path $taskOutput 'runtime_parameters.exe')
if ($LASTEXITCODE -ne 0) { throw 'Runtime tests failed' }
python -B calibrate_model.py --self-test
if ($LASTEXITCODE -ne 0) { throw 'Calibration unit tests failed' }
python -B tests/verify_firmware_logs.py $taskOutput
if ($LASTEXITCODE -ne 0) { throw 'Firmware simulation/identification failed' }

Set-Location -LiteralPath (Join-Path $taskProject 'obj')
$env:PATH = $WchBin + ';' + $env:PATH
& $Make -j4 main-build
if ($LASTEXITCODE -ne 0) { throw 'Normal firmware build failed' }
$taskCalOutput = Join-Path $taskOutput 'calibration_firmware'
New-Item -ItemType Directory -Path $taskCalOutput -Force | Out-Null
$taskFlags = @('-march=rv32ec_zmmul_xw','-mabi=ilp32e','-msmall-data-limit=0','-msave-restore','-Os','-std=gnu99','-ffunction-sections','-fdata-sections','-fno-common','-Wall','-Wextra','-Wformat=2')
foreach ($taskDir in @('SRC/Core','SRC/Debug','SRC/Peripheral/inc','User','System','Drivers','Common','Application')) {
    $taskFlags += '-I' + (Join-Path $taskProject $taskDir)
}
foreach ($taskSrc in @('Application/A_Calib_Min.c','Application/A_Config.c','Application/A_Servo.c','Application/A_UartCmd.c','Common/C_Pos_Ctrl.c','Common/C_Traj_Planner.c','Common/C_Setpoint.c')) {
    & (Join-Path $WchBin 'riscv-wch-elf-gcc.exe') @taskFlags -c (Join-Path $taskProject $taskSrc) -o (Join-Path $taskCalOutput ((Split-Path $taskSrc -Leaf) + '.o'))
    if ($LASTEXITCODE -ne 0) { throw "$taskSrc strict compile failed" }
}
$taskMain = Join-Path $taskCalOutput 'main_calibration.o'
& (Join-Path $WchBin 'riscv-wch-elf-gcc.exe') @taskFlags -DAPP_MODE_CALIB -c (Join-Path $taskProject 'User/main.c') -o $taskMain
if ($LASTEXITCODE -ne 0) { throw 'Calibration main compile failed' }
$taskObjects = @()
foreach ($taskMk in (Get-ChildItem -Recurse -Filter subdir.mk -File)) {
    $taskReading = $false
    foreach ($taskLine in [IO.File]::ReadLines($taskMk.FullName)) {
        if ($taskLine -match '^OBJS\s*\+=') { $taskReading = $true; continue }
        if (!$taskReading) { continue }
        $taskObject = $taskLine.Trim().TrimEnd('\').Trim()
        if (!$taskObject) { $taskReading = $false; continue }
        $taskObjects += $(if ($taskObject -eq './User/main.o') { $taskMain } else { $taskObject })
    }
}
$taskElf = Join-Path $taskCalOutput 'CH32V00X_CALIB.elf'
$taskLink = @('-march=rv32ec_zmmul_xw','-mabi=ilp32e','-msmall-data-limit=0','-msave-restore','-Os','-T',(Join-Path $taskProject 'SRC/Ld/Link.ld'),'-nostartfiles','-Wl,--gc-sections',('-Wl,-Map,' + (Join-Path $taskCalOutput 'CH32V00X_CALIB.map')),'--specs=nano.specs','--specs=nosys.specs','-o',$taskElf) + $taskObjects
& (Join-Path $WchBin 'riscv-wch-elf-gcc.exe') @taskLink
if ($LASTEXITCODE -ne 0) { throw 'Calibration link failed' }
& (Join-Path $WchBin 'riscv-wch-elf-size.exe') $taskElf
foreach ($taskFormat in @('ihex','binary')) {
    $taskExtension = $(if ($taskFormat -eq 'ihex') { 'hex' } else { 'bin' })
    & (Join-Path $WchBin 'riscv-wch-elf-objcopy.exe') -O $taskFormat $taskElf (Join-Path $taskCalOutput ('CH32V00X_CALIB.' + $taskExtension))
    if ($LASTEXITCODE -ne 0) { throw 'Calibration export failed' }
}
'PASS: host tests, complete calibration simulations and normal/calibration firmware builds'
