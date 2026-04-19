@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
REM ========================================
REM Source Sync Script v2
REM D:\race (VSCode workspace) -> E:\ADS (build workspace)
REM ========================================

set "SRC_DIR=D:\race\save\4.10 - vs\new_ins"
set "DST_DIR=E:\Aorcheved_01\new_ins"

set DRY_RUN=0
set VERBOSE=0
set FORCE=0
set STATS_COPIED=0
set STATS_SKIPPED=0
set STATS_UPDATED=0

:parse_args
if "%~1"=="" goto end_parse
if /i "%~1"=="--dry-run" set DRY_RUN=1
if /i "%~1"=="-n" set DRY_RUN=1
if /i "%~1"=="--verbose" set VERBOSE=1
if /i "%~1"=="-v" set VERBOSE=1
if /i "%~1"=="--force" set FORCE=1
if /i "%~1"=="-f" set FORCE=1
shift
goto parse_args
:end_parse

echo ========================================
echo Source Sync: VSCode -^> ADS v2
echo ========================================
echo Source : %SRC_DIR%
echo Target : %DST_DIR%
if %DRY_RUN% equ 1 (echo Mode    : DRY RUN) else (echo Mode    : SYNC)
if %FORCE% equ 1 (echo Overwrite: FORCE) else (echo Overwrite: safe)
echo ========================================
echo.

if not exist "%SRC_DIR%" (
    echo [ERROR] Source not found: %SRC_DIR%
    pause & exit /b 1
)

if not exist "%DST_DIR%" (
    echo [ERROR] Target not found: %DST_DIR%
    pause & exit /b 1
)

REM --- Root config files ---
echo [INFO] Checking root config files...
call :sync_one_file ".cproject"
call :sync_one_file ".project"
call :sync_one_file "Lcf_Tasking_Tricore_Tc.lsl"

REM --- Source directories ---
for %%D in (code user libraries) do (
    if exist "%SRC_DIR%\%%D" (
        echo.
        echo [INFO] Scanning %%D\
        call :sync_folder "%%D"
    )
)

REM --- Summary ---
echo.
echo ========================================
echo   Copied  : %STATS_COPIED%
echo   Updated : %STATS_UPDATED%
echo   Skipped : %STATS_SKIPPED%
echo ========================================
echo.

if %DRY_RUN% equ 1 (
    echo [INFO] Dry run - no changes made
) else (
    if %STATS_UPDATED% gtr 0 (
        echo [DONE] Synced to ADS workspace!
        echo Next: Build in ADS IDE -^> flash
    ) else (
        echo [INFO] All up to date
    )
)
pause
exit /b 0


REM ================================
REM Sync single root file
REM ================================
:sync_one_file
set "FNAME=%~1"
set "SRC=%SRC_DIR%\%FNAME%"
set "DST=%DST_DIR%\%FNAME%"

if not exist "%SRC%" exit /b

if not exist "%DST%" (
    if %DRY_RUN% equ 1 (
        echo   [NEW]     %FNAME%
    ) else (
        copy /y "%SRC%" "%DST%" >nul
        echo   [COPY]    %FNAME%
    )
    set /a STATS_COPIED+=1
    exit /b
)

for %%A in ("%SRC%") set SSize=%%~zA
for %%A in ("%DST%") set DSize=%%~zA

if %SSize% neq %DSize% (
    if %FORCE% equ 1 (
        if %DRY_RUN% equ 1 (
            echo   [UPDATE]  %FNAME% (!SSize! vs !DSize! bytes)
        ) else (
            copy /y "%SRC%" "%DST%" >nul
            echo   [UPDATE]  %FNAME%
        )
        set /a STATS_UPDATED+=1
    ) else (
        echo   [DIFF]    %FNAME% (!SSize! vs !DSize! bytes) use -f
        set /a STATS_SKIPPED+=1
    )
    exit /b
)

if %VERBOSE% equ 1 echo   [OK]      %FNAME%
set /a STATS_SKIPPED+=1
exit /b


REM ================================
REM Sync entire folder (.c + .h only)
REM ================================
:sync_folder
set "DIRNAME=%~1"
set "SBASE=%SRC_DIR%\%DIRNAME%"
set "DBASE=%DST_DIR%\%DIRNAME%"

if not exist "%DBASE%" mkdir "%DBASE%" >nul

for /r "%SBASE%" %%F in (*.c *.h) do (
    set "REL=%%F"
    set "REL=!REL:%SBASE%=!"
    set "SRCF=%%F"
    set "DSTF=%DBASE%!REL!"

    if not exist "!DSTF!" (
        if %DRY_RUN% equ 1 (
            echo   [NEW]     !DIRNAME!!REL!
        ) else (
            copy /y "!SRCF!" "!DSTF!" >nul
            echo   [COPY]    !DIRNAME!!REL!
        )
        set /a STATS_COPIED+=1
    ) else (
        for %%A in ("!SRCF!") set SZ1=%%~zA
        for %%A in ("!DSTF!") set SZ2=%%~zA

        if !SZ1! neq !SZ2! (
            if %FORCE% equ 1 (
                if %DRY_RUN% equ 1 (
                    echo   [UPDATE]  !DIRNAME!!REL! (!SZ1! vs !SZ2!)
                ) else (
                    copy /y "!SRCF!" "!DSTF!" >nul
                    echo   [UPDATE]  !DIRNAME!!REL!
                )
                set /a STATS_UPDATED+=1
            ) else (
                echo   [DIFF]    !DIRNAME!!REL! (!SZ1! vs !SZ2!)
                set /a STATS_SKIPPED+=1
            )
        ) else (
            if %VERBOSE% equ 1 echo   [OK]      !DIRNAME!!REL!
            set /a STATS_SKIPPED+=1
        )
    )
)
exit /b
