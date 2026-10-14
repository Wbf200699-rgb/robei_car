@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
set "VIVADO=D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat"

if "%~1"=="" (
  echo ERROR: usage: vivado_program_bitstream.bat path-to-bitstream
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

call "%VIVADO%" -mode batch -nojournal -nolog -source "%~dp0vivado_program_bitstream.tcl" -tclargs "%~f1"
exit /b %ERRORLEVEL%
