################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/Motor.c \
../code/PID.c \
../code/encoder.c \
../code/init.c \
../code/ins_new_264.c \
../code/menu.c \
../code/servo.c \
../code/speaker.c \
../code/speech.c \
../code/task.c \
../code/track.c \
../code/zf_device_dot_matrix_screen.c \
../code/zf_device_tld7002.c 

COMPILED_SRCS += \
code/Motor.src \
code/PID.src \
code/encoder.src \
code/init.src \
code/ins_new_264.src \
code/menu.src \
code/servo.src \
code/speaker.src \
code/speech.src \
code/task.src \
code/track.src \
code/zf_device_dot_matrix_screen.src \
code/zf_device_tld7002.src 

C_DEPS += \
code/Motor.d \
code/PID.d \
code/encoder.d \
code/init.d \
code/ins_new_264.d \
code/menu.d \
code/servo.d \
code/speaker.d \
code/speech.d \
code/task.d \
code/track.d \
code/zf_device_dot_matrix_screen.d \
code/zf_device_tld7002.d 

OBJS += \
code/Motor.o \
code/PID.o \
code/encoder.o \
code/init.o \
code/ins_new_264.o \
code/menu.o \
code/servo.o \
code/speaker.o \
code/speech.o \
code/task.o \
code/track.o \
code/zf_device_dot_matrix_screen.o \
code/zf_device_tld7002.o 


# Each subdirectory must supply rules for building sources it contributes
code/Motor.src: ../code/Motor.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/Motor.o: code/Motor.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/PID.src: ../code/PID.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/PID.o: code/PID.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/encoder.src: ../code/encoder.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/encoder.o: code/encoder.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/init.src: ../code/init.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/init.o: code/init.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/ins_new_264.src: ../code/ins_new_264.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/ins_new_264.o: code/ins_new_264.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/menu.src: ../code/menu.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/menu.o: code/menu.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/servo.src: ../code/servo.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/servo.o: code/servo.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/speaker.src: ../code/speaker.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/speaker.o: code/speaker.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/speech.src: ../code/speech.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/speech.o: code/speech.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/task.src: ../code/task.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/task.o: code/task.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/track.src: ../code/track.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/track.o: code/track.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/zf_device_dot_matrix_screen.src: ../code/zf_device_dot_matrix_screen.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/zf_device_dot_matrix_screen.o: code/zf_device_dot_matrix_screen.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/zf_device_tld7002.src: ../code/zf_device_tld7002.c code/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/TC377_Project1/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/zf_device_tld7002.o: code/zf_device_tld7002.src code/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) code/Motor.d code/Motor.o code/Motor.src code/PID.d code/PID.o code/PID.src code/encoder.d code/encoder.o code/encoder.src code/init.d code/init.o code/init.src code/ins_new_264.d code/ins_new_264.o code/ins_new_264.src code/menu.d code/menu.o code/menu.src code/servo.d code/servo.o code/servo.src code/speaker.d code/speaker.o code/speaker.src code/speech.d code/speech.o code/speech.src code/task.d code/task.o code/task.src code/track.d code/track.o code/track.src code/zf_device_dot_matrix_screen.d code/zf_device_dot_matrix_screen.o code/zf_device_dot_matrix_screen.src code/zf_device_tld7002.d code/zf_device_tld7002.o code/zf_device_tld7002.src

.PHONY: clean-code

