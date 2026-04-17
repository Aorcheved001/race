################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/system/init_all.c \
../code/system/interrupt.c \
../code/system/menu_lzj.c 

COMPILED_SRCS += \
code/system/init_all.src \
code/system/interrupt.src \
code/system/menu_lzj.src 

C_DEPS += \
code/system/init_all.d \
code/system/interrupt.d \
code/system/menu_lzj.d 

OBJS += \
code/system/init_all.o \
code/system/interrupt.o \
code/system/menu_lzj.o 


# Each subdirectory must supply rules for building sources it contributes
code/system/init_all.src: ../code/system/init_all.c code/system/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/system/init_all.o: code/system/init_all.src code/system/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/system/interrupt.src: ../code/system/interrupt.c code/system/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/system/interrupt.o: code/system/interrupt.src code/system/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/system/menu_lzj.src: ../code/system/menu_lzj.c code/system/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/system/menu_lzj.o: code/system/menu_lzj.src code/system/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-system

clean-code-2f-system:
	-$(RM) code/system/init_all.d code/system/init_all.o code/system/init_all.src code/system/interrupt.d code/system/interrupt.o code/system/interrupt.src code/system/menu_lzj.d code/system/menu_lzj.o code/system/menu_lzj.src

.PHONY: clean-code-2f-system

