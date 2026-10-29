param([string]$CubeIdePath='E:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE')
$ErrorActionPreference='Stop'
$projectDir=$PSScriptRoot
$tool=Get-ChildItem -LiteralPath (Join-Path $CubeIdePath 'plugins') -Directory |
  Where-Object Name -Like 'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*.win32_*' |
  Sort-Object Name -Descending | Select-Object -First 1
if($null -eq $tool){throw 'ARM compiler not found'}
$compiler=Join-Path $tool.FullName 'tools\bin\arm-none-eabi-gcc.exe'
$outputDir=Join-Path $projectDir 'build'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$includePaths=@('Core/Inc','Drivers/STM32F4xx_HAL_Driver/Inc','Drivers/STM32F4xx_HAL_Driver/Inc/Legacy',
  'Drivers/CMSIS/Device/ST/STM32F4xx/Include','Drivers/CMSIS/Include',
  'Middlewares/Third_Party/FreeRTOS/Source/include','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2',
  'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F')
$flags=@('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-std=gnu11','-Og','-g3',
  '-DUSE_HAL_DRIVER','-DSTM32F407xx','-DDEBUG','-ffunction-sections','-fdata-sections','-fstack-usage',
  '-Wall','-Wextra','-Werror')
foreach($includePath in $includePaths){$flags+=('-I'+(Join-Path $projectDir $includePath))}
$sources=@(Get-ChildItem -LiteralPath (Join-Path $projectDir 'Core/Src') -Filter '*.c' -File)
$sources+=Get-ChildItem -LiteralPath (Join-Path $projectDir 'Drivers/STM32F4xx_HAL_Driver/Src') -Filter '*.c' -File
$rtos=Join-Path $projectDir 'Middlewares/Third_Party/FreeRTOS/Source'
$sources+=Get-ChildItem -LiteralPath $rtos -Filter '*.c' -File
$sources+=Get-Item -LiteralPath (Join-Path $rtos 'CMSIS_RTOS_V2/cmsis_os2.c'),
  (Join-Path $rtos 'portable/GCC/ARM_CM4F/port.c'),(Join-Path $rtos 'portable/MemMang/heap_4.c')
$objects=@()
foreach($source in $sources){
  $relative=$source.FullName.Substring($projectDir.Length+1) -replace '[\\/:]','_'
  $obj=Join-Path $outputDir ($relative+'.o')
  $sourceFlags=$flags
  if($source.FullName -notlike (Join-Path $projectDir 'Core\*')) {
    $sourceFlags=@($flags | Where-Object {$_ -ne '-Wextra'})
  }
  & $compiler @sourceFlags -c $source.FullName -o $obj
  if($LASTEXITCODE -ne 0){throw ('Compile failed: '+$source.FullName)}
  $objects+=$obj
}
$startup=Join-Path $projectDir 'Core/Startup/startup_stm32f407zgtx.s'
$startupObj=Join-Path $outputDir 'startup.o'
& $compiler -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -x assembler-with-cpp -c $startup -o $startupObj
if($LASTEXITCODE -ne 0){throw 'Startup compile failed'}
$objects+=$startupObj
$elf=Join-Path $outputDir 'H2026_LineCar.elf'
$map=Join-Path $outputDir 'H2026_LineCar.map'
$link=@('-mcpu=cortex-m4','-mthumb','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','--specs=nano.specs','--specs=nosys.specs',
  ('-T'+(Join-Path $projectDir 'STM32F407ZGTX_FLASH.ld')),'-Wl,--gc-sections',('-Wl,-Map='+$map),
  '-Wl,--start-group','-lc','-lm','-Wl,--end-group')
& $compiler @objects @link -o $elf
if($LASTEXITCODE -ne 0){throw 'Link failed'}
$binDir=Join-Path $tool.FullName 'tools/bin'
& (Join-Path $binDir 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $outputDir 'H2026_LineCar.hex')
if($LASTEXITCODE -ne 0){throw 'HEX export failed'}
& (Join-Path $binDir 'arm-none-eabi-objcopy.exe') -O binary $elf (Join-Path $outputDir 'H2026_LineCar.bin')
if($LASTEXITCODE -ne 0){throw 'BIN export failed'}
& (Join-Path $binDir 'arm-none-eabi-size.exe') $elf
if($LASTEXITCODE -ne 0){throw 'Size check failed'}
Write-Output ('Firmware: '+(Join-Path $outputDir 'H2026_LineCar.hex'))
