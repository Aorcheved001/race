@echo off
chcp 65001 >nul
REM ========================================
REM AURIX Studio Build ^& Flash Script v9
REM Sync + Build + Flash - All-in-One
REM ========================================

set ADS_DIR=E:\Aorcheved_01\AURIX-Studio-1.10.2
set PROJECT_DIR=E:\Aorcheved_01\new_ins
set BUILD_DIR=%PROJECT_DIR%\Debug
set TRAE_DIR=D:\race\save\4.10\new_ins

set MAKE_EXE=%ADS_DIR%\tools\make\make.exe
set TOOLCHAIN_BIN=%ADS_DIR%\tools\Compilers\Tasking_1.1r8\ctc\bin
set FLASHER_EXE=%ADS_DIR%\tools\AurixFlasherSoftwareTool_v1.0.8\AurixFlasher.exe
set SYNC_PY=%TRAE_DIR%\tools\sync.py

set ELF_FILE=%BUILD_DIR%\new_ins.elf
set HEX_FILE=%BUILD_DIR%\new_ins.hex

set MODE=0
set DO_SYNC=0
set FLASH_ERASE=on
set FLASH_VERIFY=on
set FLASH_CONNECT=6
set FLASH_START=on
set NO_CLEAN=0

:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="-flash" set MODE=2
if /i "%~1"=="-f" set MODE=2
if /i "%~1"=="--flash-only" set MODE=1
if /i "%~1"=="--flash-only" set MODE=1
if /i "%~1"=="-no-flash" set MODE=0
if /i "%~1"=="--sync" set DO_SYNC=1
if /i "%~1"=="-s" set DO_SYNC=1
if /i "%~1"=="-erase-all" set FLASH_ERASE=all
if /i "%~1"=="-no-verify" set FLASH_VERIFY=off
if /i "%~1"=="-no-start" set FLASH_START=off
if /i "%~1"=="--no-clean" set NO_CLEAN=1
shift
goto parse_args
:end_parse

echo ========================================
echo AURIX Studio Build ^& Flash v9
echo ========================================
echo ADS Dir : %ADS_DIR%
echo Trae   : %TRAE_DIR%
echo Project: %PROJECT_DIR%
echo Build  : %BUILD_DIR%

if %MODE% equ 0 (
    echo Mode   : BUILD ONLY
) else if %MODE% equ 1 (
    echo Mode   : FLASH ONLY
) else (
    echo Mode   : BUILD + FLASH
)

if %DO_SYNC% equ 1 (echo Sync   : ON) else (echo Sync   : OFF)
echo ========================================
echo.

REM ================================
REM SYNC PHASE (optional)
REM ================================
if %DO_SYNC% equ 1 (
    if exist "%SYNC_PY%" (
        echo [INFO] === Syncing Trae -^> ADS ===
        python "%SYNC_PY%" --force
        echo.
    ) else (
        echo [WARN] sync.py not found at %SYNC_PY%
        echo [WARN] Skipping sync...
    )
)

cd /d "%BUILD_DIR%"
set PATH=%TOOLCHAIN_BIN%;%PATH%

echo @echo off > "%TEMP%\elfsize.bat"
echo echo [INFO] ELF Size: %%~nx1 >> "%TEMP%\elfsize.bat"
echo dir "%%1" >> "%TEMP%\elf%"
set PATH=%TEMP%;%PATH%

REM ================================
REM BUILD PHASE (mode 0 or 2)
REM ================================
if %MODE% equ 1 goto flash_phase

if not exist "%MAKE_EXE%" (
    echo [ERROR] make not found: %MAKE_EXE%
    pause & exit /b 1
)
if not exist "%BUILD_DIR%\makefile" (
    echo [ERROR] makefile not found: %BUILD_DIR%\makefile%
    pause & exit /b 1
)

if %NO_CLEAN% equ 0 (
    echo [INFO] Cleaning old artifacts...
    if exist "*.elf" del /q *.elf 2>nul
    if exist "*.hex" del /q *.hex 2>nul
    if exist "*.map" del /q *.map 2>nul
) else (
    echo [INFO] Incremental build (--no-clean)
)

echo.
echo [INFO] Starting build...
echo.

"%MAKE_EXE%" -C "%BUILD_DIR%" -k all
set BUILD_RESULT=%ERRORLEVEL%

echo.
if not exist "%ELF_FILE%" (
    echo [ERROR] Build failed - ELF not generated
    echo Error code: %BUILD_RESULT%
    echo.
    echo NOTE: Tasking non-commercial license prevents standalone linking.
    echo       Solution:
    echo         1. Build in ADS IDE (Ctrl+B)
    echo         2. Then use: headless_build.bat -s --flash-only
    goto cleanup
)

echo ========================================
echo [SUCCESS] Build completed!
echo ========================================
echo Output files:
echo   + ELF: %ELF_FILE%
if exist "%HEX_FILE%" echo   + HEX: %HEX_FILE%
if exist "%BUILD_DIR%\new_ins.map" echo   + MAP: %BUILD_DIR%\new_ins.map

echo.
for %%F in ("%ELF_FILE%") do echo   ELF: %%~zF bytes
for %%F in ("%HEX_FILE%") do echo   HEX: %%~zF bytes

if %MODE% equ 0 (
    echo.
    echo [INFO] Flash skipped (use -f to enable, -s to sync first)
    goto cleanup
)

REM ================================
REM FLASH PHASE (mode 1 or 2)
REM ================================
:flash_phase

if not exist "%ELF_FILE%" (
    echo [ERROR] Firmware not found: %ELF_FILE%
    echo Please build first (ADS IDE Ctrl+B)
    goto cleanup
)

echo.
echo ========================================
echo [INFO] Starting flash...
echo ========================================

if not exist "%FLASHER_EXE%" (
    echo [ERROR] AurixFlasher not found: %FLASHER_EXE%
    goto cleanup
)

set FLASH_LOG=%BUILD_DIR%\flash_log.xml

echo [INFO] Flasher : %FLASHER_EXE%
echo [INFO] Firmware: %ELF_FILE%
echo [INFO] Erase   : %FLASH_ERASE%
echo [INFO] Verify  : %FLASH_VERIFY%
echo [INFO] Connect : %FLASH_CONNECT% (reset^&halt)
echo [INFO] Start   : %FLASH_START%
echo [INFO] Log     : %FLASH_LOG%
echo.

"%FLASHER_EXE%" ^
    -elf "%ELF_FILE%" ^
    -erase %FLASH_ERASE% ^
    -ver %FLASH_VERIFY% ^
    -connect %FLASH_CONNECT% ^
    -start %FLASH_START% ^
    -log "%FLASH_LOG%"

set FLASH_RESULT=%ERRORLEVEL%

echo.
if %FLASH_RESULT% equ 0 (
    echo ========================================
    echo [SUCCESS] Flash completed!
    if %FLASH_START% equ on (
        echo Device is now running
    ) else (
        echo Device in HALT state
    )
    echo ========================================
) else (
    echo [ERROR] Flash failed, code: %FLASH_RESULT%
    echo.
    echo Possible causes:
    echo   1. Debugger not connected
    echo   2. Device power off
    echo   3. Check %FLASH_LOG% for details
)

:cleanup
del /q "%TEMP%\elfsize.bat" 2>nul
echo.
pause
exit /b %BUILD_RESULT%
