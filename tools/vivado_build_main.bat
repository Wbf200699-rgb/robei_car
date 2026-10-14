@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
if not defined VIVADO (
  for /f "delims=" %%I in ('where vivado.bat 2^>nul') do set "VIVADO=%%I"
)

if not exist "%VIVADO%" (
  echo ERROR: Set VIVADO to your vivado.bat path, or add Vivado to PATH.
  exit /b 2
)
if not exist "%~dp0..\little_car\build\filelist.vf" (
  echo ERROR: little_car\build\filelist.vf is missing
  exit /b 2
)
if not exist "%~dp0..\little_car\build\top.xdc" (
  echo ERROR: little_car\build\top.xdc is missing
  exit /b 2
)

pushd "%~dp0.."
call "%VIVADO%" -mode batch -nojournal -nolog -source "%~dp0vivado_build_main.tcl"
set VIVADO_EXIT=%ERRORLEVEL%
if exist "dfx_runtime.txt" del /f /q "dfx_runtime.txt" >nul 2>nul
popd
exit /b %VIVADO_EXIT%
