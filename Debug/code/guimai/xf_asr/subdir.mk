################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../code/guimai/xf_asr/asr_audio.c \
../code/guimai/xf_asr/base64.c \
../code/guimai/xf_asr/hmac_sha256.c \
../code/guimai/xf_asr/sha1.c \
../code/guimai/xf_asr/websocket_client.c 

COMPILED_SRCS += \
code/guimai/xf_asr/asr_audio.src \
code/guimai/xf_asr/base64.src \
code/guimai/xf_asr/hmac_sha256.src \
code/guimai/xf_asr/sha1.src \
code/guimai/xf_asr/websocket_client.src 

C_DEPS += \
code/guimai/xf_asr/asr_audio.d \
code/guimai/xf_asr/base64.d \
code/guimai/xf_asr/hmac_sha256.d \
code/guimai/xf_asr/sha1.d \
code/guimai/xf_asr/websocket_client.d 

OBJS += \
code/guimai/xf_asr/asr_audio.o \
code/guimai/xf_asr/base64.o \
code/guimai/xf_asr/hmac_sha256.o \
code/guimai/xf_asr/sha1.o \
code/guimai/xf_asr/websocket_client.o 


# Each subdirectory must supply rules for building sources it contributes
code/guimai/xf_asr/asr_audio.src: ../code/guimai/xf_asr/asr_audio.c code/guimai/xf_asr/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/guimai/xf_asr/asr_audio.o: code/guimai/xf_asr/asr_audio.src code/guimai/xf_asr/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/guimai/xf_asr/base64.src: ../code/guimai/xf_asr/base64.c code/guimai/xf_asr/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/guimai/xf_asr/base64.o: code/guimai/xf_asr/base64.src code/guimai/xf_asr/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/guimai/xf_asr/hmac_sha256.src: ../code/guimai/xf_asr/hmac_sha256.c code/guimai/xf_asr/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/guimai/xf_asr/hmac_sha256.o: code/guimai/xf_asr/hmac_sha256.src code/guimai/xf_asr/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/guimai/xf_asr/sha1.src: ../code/guimai/xf_asr/sha1.c code/guimai/xf_asr/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/guimai/xf_asr/sha1.o: code/guimai/xf_asr/sha1.src code/guimai/xf_asr/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
code/guimai/xf_asr/websocket_client.src: ../code/guimai/xf_asr/websocket_client.c code/guimai/xf_asr/subdir.mk
	cctc -cs --dep-file="$(*F).d" --misrac-version=2004 -D__CPU__=tc37x "-fD:/AURIX-v1.10.2-workspace/Seekfree_TC377_Opensource_Library/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
code/guimai/xf_asr/websocket_client.o: code/guimai/xf_asr/websocket_client.src code/guimai/xf_asr/subdir.mk
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code-2f-guimai-2f-xf_asr

clean-code-2f-guimai-2f-xf_asr:
	-$(RM) code/guimai/xf_asr/asr_audio.d code/guimai/xf_asr/asr_audio.o code/guimai/xf_asr/asr_audio.src code/guimai/xf_asr/base64.d code/guimai/xf_asr/base64.o code/guimai/xf_asr/base64.src code/guimai/xf_asr/hmac_sha256.d code/guimai/xf_asr/hmac_sha256.o code/guimai/xf_asr/hmac_sha256.src code/guimai/xf_asr/sha1.d code/guimai/xf_asr/sha1.o code/guimai/xf_asr/sha1.src code/guimai/xf_asr/websocket_client.d code/guimai/xf_asr/websocket_client.o code/guimai/xf_asr/websocket_client.src

.PHONY: clean-code-2f-guimai-2f-xf_asr

