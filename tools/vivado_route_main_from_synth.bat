@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
set "VIVADO=D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat"

if not exist "%VIVADO%" (
  echo ERROR: Vivado was not found at %VIVADO%
  exit /b 2
)
if not exist "%~dp0..\little_car\build\vivado_main\post_synth.dcp" (
  echo ERROR: post_synth.dcp is missing
  exit /b 2
)

pushd "%~dp0.."
call "%VIVADO%" -mode batch -nojournal -nolog -source "%~dp0vivado_route_main_from_synth.tcl"
set VIVADO_EXIT=%ERRORLEVEL%
if exist "dfx_runtime.txt" del /f /q "dfx_runtime.txt" >nul 2>nul
popd
exit /b %VIVADO_EXIT%
