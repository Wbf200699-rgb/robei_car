@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
set "VIVADO=D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat"

if "%~1"=="" (
  echo ERROR: usage: vivado_program_cfgmem.bat path-to-bitstream [cfgmem-part]
  exit /b 2
)
if not exist "%VIVADO%" (
  echo ERROR: Vivado was not found at %VIVADO%
  exit /b 2
)
if not exist "%~1" (
  echo ERROR: bitstream does not exist: %~1
  exit /b 2
)

if "%~2"=="" (
  call "%VIVADO%" -mode batch -nojournal -nolog -source "%~dp0vivado_program_cfgmem.tcl" -tclargs "%~f1"
) else (
  call "%VIVADO%" -mode batch -nojournal -nolog -source "%~dp0vivado_program_cfgmem.tcl" -tclargs "%~f1" "%~2"
)
exit /b %ERRORLEVEL%
