@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
set "VIVADO=D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat"
if not exist "%VIVADO%" set "VIVADO=D:\Apps\Vivado\2025.2\bin\vivado.bat"
if not exist "%VIVADO%" (
  echo ERROR: Vivado was not found under D:\Apps\Vivado
  exit /b 2
)
call "%VIVADO%" -mode batch -nojournal -nolog -source "%~dp0vivado_measure_leaf_resources.tcl"
exit /b %ERRORLEVEL%
