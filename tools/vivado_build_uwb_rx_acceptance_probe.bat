@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
set "VIVADO=D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat"
if not exist "%VIVADO%" (
  echo ERROR: Vivado was not found at %VIVADO%
  exit /b 1
)

pushd "%~dp0.."
call "%VIVADO%" -mode batch -source "%~dp0vivado_build_uwb_rx_acceptance_probe.tcl" -nojournal -nolog
set VIVADO_EXIT=%ERRORLEVEL%
if exist "dfx_runtime.txt" del /f /q "dfx_runtime.txt" >nul 2>nul
popd
exit /b %VIVADO_EXIT%
