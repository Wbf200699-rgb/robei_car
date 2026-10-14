@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
pushd "%~dp0.."
call "D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat" -mode batch -source "%~dp0vivado_check_c5.tcl" -nojournal -nolog
set VIVADO_EXIT=%ERRORLEVEL%
if exist "dfx_runtime.txt" del /f /q "dfx_runtime.txt" >nul 2>nul
popd
exit /b %VIVADO_EXIT%
