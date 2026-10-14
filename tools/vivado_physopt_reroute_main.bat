@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
pushd "%~dp0.."
call "D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat" -mode batch -nojournal -nolog -source "%~dp0vivado_physopt_reroute_main.tcl"
set VIVADO_EXIT=%ERRORLEVEL%
popd
exit /b %VIVADO_EXIT%
