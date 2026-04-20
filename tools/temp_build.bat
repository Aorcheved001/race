@echo off
REM ========================================
REM AURIX Studio Headless Build Script
REM 使用临时工作区避免权限问题
REM ========================================

REM 设置 ADS 安装路径
set ADS_DIR=E:\Aorcheved_01\AURIX-Studio-1.10.2

REM 设置 Java 路径（使用 ADS 自带的 JRE）
set JAVA_EXE=%ADS_DIR%\plugins\org.eclipse.justj.openjdk.hotspot.jre.full.win32.x86_64_17.0.4.v20221004-1257\jre\bin\java.exe

REM 如果 ADS 自带 JRE 不存在，尝试使用系统 Java
if not exist "%JAVA_EXE%" (
    echo [警告] ADS 自带 JRE 不存在，尝试使用系统 Java
    set JAVA_EXE=java
)

REM Eclipse Equinox Launcher JAR
set LAUNCHER_JAR=%ADS_DIR%\plugins\org.eclipse.equinox.launcher_1.6.400.v20210924-0641.jar

REM 检查必要文件是否存在
if not exist "%JAVA_EXE%" (
    echo [错误] 找不到 Java: %JAVA_EXE%
    echo 请确保已安装 Java 或 ADS 自带 JRE 存在
    pause
    exit /b 1
)

if not exist "%LAUNCHER_JAR%" (
    echo [错误] 找不到 Equinox Launcher: %LAUNCHER_JAR%
    pause
    exit /b 1
)

REM 创建临时工作区和项目目录
set TEMP_WORKSPACE=%TEMP%\aurix_temp_workspace
set TEMP_PROJECT=%TEMP_WORKSPACE%\new_ins

REM 清理并创建临时目录
rmdir /s /q "%TEMP_WORKSPACE%" 2>nul
mkdir "%TEMP_PROJECT%" 2>nul

REM 复制项目文件到临时目录
echo [信息] 复制项目文件到临时工作区...
copy "*.cproject" "%TEMP_PROJECT%" 2>nul
copy "*.project" "%TEMP_PROJECT%" 2>nul
xcopy "code" "%TEMP_PROJECT%\code" /s /e /y 2>nul
xcopy "libraries" "%TEMP_PROJECT%\libraries" /s /e /y 2>nul
xcopy "user" "%TEMP_PROJECT%\user" /s /e /y 2>nul
xcopy ".settings" "%TEMP_PROJECT%\.settings" /s /e /y 2>nul

REM 创建临时配置目录
set TEMP_CONFIG_DIR=%TEMP%\aurix_headless_config
mkdir "%TEMP_CONFIG_DIR%" 2>nul

echo ========================================
echo AURIX Studio Headless Build
echo ========================================
echo ADS 目录：%ADS_DIR%
echo 临时工作区：%TEMP_WORKSPACE%
echo 项目名：new_ins
echo Java: %JAVA_EXE%
echo ========================================
echo.

REM 执行 clean build（先清理再编译）
echo [信息] 开始清理并构建项目...
"%JAVA_EXE%" ^
    -jar "%LAUNCHER_JAR%" ^
    -application org.eclipse.cdt.managedbuilder.core.headlessbuild ^
    -data "%TEMP_WORKSPACE%" ^
    -configuration "%TEMP_CONFIG_DIR%" ^
    -cleanBuild "new_ins" ^
    -clean ^
    -consoleLog

set BUILD_RESULT=%ERRORLEVEL%

REM 复制构建结果回原目录
if %BUILD_RESULT% equ 0 (
    echo.
    echo [信息] 构建成功，复制结果文件...
    if exist "%TEMP_PROJECT%\Debug" (
        xcopy "%TEMP_PROJECT%\Debug" "Debug" /s /e /y 2>nul
    )
)

REM 清理临时目录
rmdir /s /q "%TEMP_WORKSPACE%" 2>nul
rmdir /s /q "%TEMP_CONFIG_DIR%" 2>nul

if %BUILD_RESULT% neq 0 (
    echo.
    echo [错误] 构建失败，错误代码：%BUILD_RESULT%
    pause
    exit /b %BUILD_RESULT%
)

echo.
echo [成功] 构建完成！
pause
