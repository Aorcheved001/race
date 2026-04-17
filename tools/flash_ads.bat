@echo off 
echo ====================================== 
echo 正在烧录固件... 
echo ====================================== 

cd /d "%~dp0.." 

if not exist "Debug\new_ins.elf" ( 
    if not exist "Debug\TC377_Library.elf" ( 
        echo [错误] 未找到编译文件！请先运行 build_ads.bat 
        exit 1 
    ) 
    set ELF_FILE=Debug\TC377_Library.elf 
) else ( 
    set ELF_FILE=Debug\new_ins.elf 
) 

set FLASHER=E:\Aorcheved_01\AURIX-Studio-1.10.2\tools\AurixFlasherSoftwareTool_v1.0.8\AurixFlasher.exe 

if exist "%FLASHER%" ( 
    "%FLASHER%" -elf "%ELF_FILE%" -device TC377TP 
) else ( 
    echo [错误] 找不到 AurixFlasher.exe！ 
    exit 1 
) 

if %errorlevel% equ 0 ( 
    echo ? 烧录成功！ 
    exit 0 
) else ( 
    echo ? 烧录失败！ 
    exit 1 
)