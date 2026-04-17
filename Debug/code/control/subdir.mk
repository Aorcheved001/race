################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/control/Motor.c \
../code/control/PID.c \
../code/control/encoder.c \
../code/control/ins_new_264.c \
../code/control/servo.c \
../code/control/steering_control.c \
../code/control/track.c \
../code/control/yaokong.c \
../code/control/zf_device_lora3a22.c 

COMPILED_SRCS += \
code/control/Motor.src \
code/control/PID.src \
code/control/encoder.src \
code/control/ins_new_264.src \
code/control/servo.src \
code/control/steering_control.src \
code/control/track.src \
code/control/yaokong.src \
code/control/zf_device_lora3a22.src 

C_DEPS += \
code/control/Motor.d \
code/control/PID.d \
code/control/encoder.d \
code/control/ins_new_264.d \
code/control/servo.d \
code/control/steering_control.d \
code/control/track.d \
code/control/yaokong.d \
code/control/zf_device_lora3a22.d 

OBJS += \
code/control/Motor.o \
code/control/PID.o \
code/control/encoder.o \
code/control/ins_new_264.o \
code/control/servo.o \
code/control/steering_control.o \
code/control/track.o \
code/control/yaokong.o \
code/control/zf_device_lora3a22.o 


# Each subdirectory must supply rules for building sources it contributes
code/control/Motor.src: ../code/control/Motor.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/Motor.o: code/control/Motor.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/PID.src: ../code/control/PID.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/PID.o: code/control/PID.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/encoder.src: ../code/control/encoder.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/encoder.o: code/control/encoder.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/ins_new_264.src: ../code/control/ins_new_264.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/ins_new_264.o: code/control/ins_new_264.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/servo.src: ../code/control/servo.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/servo.o: code/control/servo.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/steering_control.src: ../code/control/steering_control.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/steering_control.o: code/control/steering_control.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/track.src: ../code/control/track.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/track.o: code/control/track.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/yaokong.src: ../code/control/yaokong.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/yaokong.o: code/control/yaokong.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/control/zf_device_lora3a22.src: ../code/control/zf_device_lora3a22.c code/control/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/A/AURIX-v1.10.2-workspace/new_ins/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/control/zf_device_lora3a22.o: code/control/zf_device_lora3a22.src code/control/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-control

clean-code-2f-control:
	-$(RM) code/control/Motor.d code/control/Motor.o code/control/Motor.src code/control/PID.d code/control/PID.o code/control/PID.src code/control/encoder.d code/control/encoder.o code/control/encoder.src code/control/ins_new_264.d code/control/ins_new_264.o code/control/ins_new_264.src code/control/servo.d code/control/servo.o code/control/servo.src code/control/steering_control.d code/control/steering_control.o code/control/steering_control.src code/control/track.d code/control/track.o code/control/track.src code/control/yaokong.d code/control/yaokong.o code/control/yaokong.src code/control/zf_device_lora3a22.d code/control/zf_device_lora3a22.o code/control/zf_device_lora3a22.src

.PHONY: clean-code-2f-control

