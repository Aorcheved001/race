################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/ins/Ins.c \
../code/ins/gnss.c \
../code/ins/imu660.c 

COMPILED_SRCS += \
code/ins/Ins.src \
code/ins/gnss.src \
code/ins/imu660.src 

C_DEPS += \
code/ins/Ins.d \
code/ins/gnss.d \
code/ins/imu660.d 

OBJS += \
code/ins/Ins.o \
code/ins/gnss.o \
code/ins/imu660.o 


# Each subdirectory must supply rules for building sources it contributes
code/ins/Ins.src: ../code/ins/Ins.c code/ins/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/ins/Ins.o: code/ins/Ins.src code/ins/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/ins/gnss.src: ../code/ins/gnss.c code/ins/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/ins/gnss.o: code/ins/gnss.src code/ins/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/ins/imu660.src: ../code/ins/imu660.c code/ins/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/ins/imu660.o: code/ins/imu660.src code/ins/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-ins

clean-code-2f-ins:
	-$(RM) code/ins/Ins.d code/ins/Ins.o code/ins/Ins.src code/ins/gnss.d code/ins/gnss.o code/ins/gnss.src code/ins/imu660.d code/ins/imu660.o code/ins/imu660.src

.PHONY: clean-code-2f-ins

