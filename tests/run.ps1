$ErrorActionPreference='Stop'
$projectDir=Split-Path -Parent $PSScriptRoot
$outputDir=Join-Path $projectDir 'build-host'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$testExe=Join-Path $outputDir 'test_line_control.exe'
& gcc -std=c11 -Wall -Wextra -Werror ('-I'+(Join-Path $projectDir 'Core/Inc')) `
  (Join-Path $PSScriptRoot 'test_line_control.c') (Join-Path $projectDir 'Core/Src/line_control.c') -lm -o $testExe
if($LASTEXITCODE -ne 0){throw 'Host test compilation failed'}
& $testExe
if($LASTEXITCODE -ne 0){throw 'Host tests failed'}
$testExe=Join-Path $outputDir 'test_line_app.exe'
$testIncludes=@(('-I'+(Join-Path $PSScriptRoot 'stubs')),('-I'+(Join-Path $projectDir 'Core/Inc')))
& gcc -std=c11 -Wall -Wextra -Werror @testIncludes (Join-Path $PSScriptRoot 'test_line_app.c') `
  (Join-Path $projectDir 'Core/Src/line_app.c') (Join-Path $projectDir 'Core/Src/line_control.c') -lm -o $testExe
if($LASTEXITCODE -ne 0){throw 'Application test compilation failed'}
& $testExe
if($LASTEXITCODE -ne 0){throw 'Application tests failed'}
$testExe=Join-Path $outputDir 'test_line_sensor.exe'
& gcc -std=c11 -Wall -Wextra -Werror @testIncludes (Join-Path $PSScriptRoot 'test_line_sensor.c') `
  (Join-Path $projectDir 'Core/Src/line_sensor.c') -o $testExe
if($LASTEXITCODE -ne 0){throw 'Sensor test compilation failed'}
& $testExe
if($LASTEXITCODE -ne 0){throw 'Sensor tests failed'}
$testExe=Join-Path $outputDir 'test_chassis.exe'
$chassisIncludes=@(('-I'+(Join-Path $PSScriptRoot 'chassis_stubs')),('-I'+(Join-Path $projectDir 'Core/Inc')))
$chassisSources=@('jy901s.c','wit_c_sdk.c','heading_pid.c','chassis_control.c','speed_pid.c','motion_profile.c') |
  ForEach-Object {Join-Path $projectDir ('Core/Src/'+$_)}
& gcc -std=c11 -Wall -Wextra -Werror -pedantic @chassisIncludes `
  (Join-Path $PSScriptRoot 'test_chassis.c') @chassisSources -lm -o $testExe
if($LASTEXITCODE -ne 0){throw 'Movement regression compilation failed'}
& $testExe
if($LASTEXITCODE -ne 0){throw 'Movement regression failed'}
