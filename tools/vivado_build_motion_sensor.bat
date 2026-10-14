@echo off
setlocal
set PROCESSOR_ARCHITECTURE=AMD64
if not defined SystemDrive set SystemDrive=C:
if not defined SystemRoot set SystemRoot=C:\Windows
if not defined ProgramData set ProgramData=%SystemDrive%\ProgramData
set "VIVADO=D:\Apps\Vivado\2025.2\Vivado\bin\vivado.bat"
set "PROFILE=%~1"
if not defined PROFILE set "PROFILE=normal"
if not exist "%VIVADO%" (
  echo ERROR: Vivado was not found at %VIVADO%
  exit /b 1
)

python "%~dp0robei_converter.py" model2v "%~dp0..\little_car\model\power_on_reset.model" -o "%~dp0..\little_car\build\power_on_reset.v"
if errorlevel 1 exit /b 1
python "%~dp0robei_converter.py" model2v "%~dp0..\little_car\model\motion_sensor_subsystem.model" -o "%~dp0..\little_car\build\motion_sensor_subsystem.v"
if errorlevel 1 exit /b 1
python "%~dp0robei_converter.py" model2v "%~dp0..\little_car\model\dual_imu_static_calibrator.model" -o "%~dp0..\little_car\build\dual_imu_static_calibrator.v"
if errorlevel 1 exit /b 1
python "%~dp0robei_converter.py" model2v "%~dp0..\little_car\model\motion_sensor_bringup_top.model" -o "%~dp0..\little_car\build\motion_sensor_bringup_top.v"
if errorlevel 1 exit /b 1

pushd "%~dp0.."
call "%VIVADO%" -mode batch -source "%~dp0vivado_build_motion_sensor.tcl" -tclargs "%PROFILE%" -nojournal -nolog
set VIVADO_EXIT=%ERRORLEVEL%
if exist "dfx_runtime.txt" del /f /q "dfx_runtime.txt" >nul 2>nul
popd
exit /b %VIVADO_EXIT%
