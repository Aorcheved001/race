@echo off
setlocal EnableDelayedExpansion

:: ================== MODIFY ONLY THESE 4 LINES ==================
set "ADS_PATH=E:\Aorcheved_01\AURIX-Studio-1.10.2"
set "WORKSPACE=D:\Aorcheved_01\new_ins"
set "PROJECT_NAME=new_ins"
set "CONFIG_NAME=com.tasking.config.TriCore_Debug"
:: ===========================================================

:: Use a separate configuration directory to avoid permission issues
set "CONFIG_DIR=%WORKSPACE%\.eclipse_config"

echo ======================================
echo Starting ADS Headless Build for AURIX project...
echo Project: %PROJECT_NAME%
echo Config:  %CONFIG_NAME%
echo Workspace: %WORKSPACE%
echo Config Dir: %CONFIG_DIR%
echo ======================================

:: Find launcher jar
for /f "delims=" %%a in ('dir /b "%ADS_PATH%\plugins\org.eclipse.equinox.launcher_*.jar" 2^>nul') do set "LAUNCHER_JAR=%%a"

:: Find java.exe
for /f "delims=" %%a in ('dir /b /s "%ADS_PATH%\*\java.exe" 2^>nul') do set "JAVA_EXE=%%a"

if not defined LAUNCHER_JAR (
    echo [ERROR] Cannot find org.eclipse.equinox.launcher_*.jar
    pause & exit /b 1
)

if not defined JAVA_EXE (
    echo [ERROR] Cannot find java.exe
    pause & exit /b 1
)

echo Using launcher: %LAUNCHER_JAR%
echo Using java: %JAVA_EXE%

:: Create config directory
if not exist "%CONFIG_DIR%" mkdir "%CONFIG_DIR%"

"%JAVA_EXE%" ^
  -Dosgi.requiredJavaVersion=17 ^
  -Xms512m -Xmx2048m ^
  -jar "%ADS_PATH%\plugins\%LAUNCHER_JAR%" ^
  -data "%WORKSPACE%" ^
  -configuration "%CONFIG_DIR%" ^
  -application org.eclipse.cdt.managedbuilder.core.headlessbuild ^
  -cleanBuild "%PROJECT_NAME%/%CONFIG_NAME%" ^
  -verbose

if %errorlevel% == 0 (
    echo ? Build succeeded! .elf file generated.
) else (
    echo ? Build failed with code: %errorlevel%
    echo Suggestions:
    echo 1. Try moving the entire project to a pure English path (e.g. D:\AURIX_Projects\new_ins) - Chinese paths often cause issues.
    echo 2. Open the project in ADS GUI first and build manually once.
    echo 3. Open .cproject file, search for "com.tasking.config" and copy the exact configuration id.
)

pause