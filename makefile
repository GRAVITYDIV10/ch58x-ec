CH58X_SDK ?= ./EVT/EXAM/SRC

SRCS += $(CH58X_SDK)/Startup/startup_CH583.S
#SRCS += $(CH58X_SDK)/../FreeRTOS/Startup/startup_CH583.S

SRCS += $(CH58X_SDK)/RVMSIS/core_riscv.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_adc.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_clk.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_flash.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_gpio.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_i2c.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_spi0.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_spi1.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_sys.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_timer0.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_timer1.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_timer2.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_timer3.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_uart0.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_uart1.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_uart2.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_uart3.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_usbdev.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_pwm.c \
	$(CH58X_SDK)/StdPeriphDriver/CH58x_pwr.c

INCS += -I $(CH58X_SDK)/RVMSIS \
	-I $(CH58X_SDK)/StdPeriphDriver/inc

LIBS += -L $(CH58X_SDK)/StdPeriphDriver \
	-lISP583

#SRCS += $(CH58X_SDK)/StdPeriphDriver/CH58x_usbhostBase.c
#SRCS += $(CH58X_SDK)/StdPeriphDriver/CH58x_usb2hostBase.c
#SRCS += $(CH58X_SDK)/StdPeriphDriver/CH58x_usbhostClass.c
#SRCS += $(CH58X_SDK)/StdPeriphDriver/CH58x_usb2hostClass.c
#SRCS += $(CH58X_SDK)/StdPeriphDriver/CH58x_usb2dev.c

#SRCS += $(CH58X_SDK)/../FreeRTOS/FreeRTOS/croutine.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/event_groups.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/list.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/queue.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/stream_buffer.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/tasks.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/timers.c \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/GCC/RISC-V/portASM.S \
#	$(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/GCC/RISC-V/port.c

#SRCS += $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/MemMang/heap_1.c
#SRCS += $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/MemMang/heap_2.c
#SRCS += $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/MemMang/heap_3.c
#SRCS += $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/MemMang/heap_4.c
#SRCS += $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/MemMang/heap_5.c

#INCS += -I $(CH58X_SDK)/../FreeRTOS/FreeRTOS \
#	-I $(CH58X_SDK)/../FreeRTOS/FreeRTOS/include \
#	-I $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/ \
#	-I $(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/GCC/RISC-V/

SRCS += main.c usbdev.c 

FW_NAME ?= ch58x-ec
# you can make toolchain by yourself:
# https://github.com/GRAVITYDIV10/gdiv10-toolchain/blob/master/crosstool-ng/riscv32-ch58x-elf.config
CROSS_COMPILE ?= riscv32-ch58x-elf-
CC = $(CROSS_COMPILE)gcc
OD = $(CROSS_COMPILE)objdump
OC = $(CROSS_COMPILE)objcopy
SZ = $(CROSS_COMPILE)size

LINK_SCRIPT ?= $(CH58X_SDK)/Ld/Link.ld
#LINK_SCRIPT ?= $(CH58X_SDK)/../FreeRTOS/Ld/Link.ld

CFLAGS += -Wall -Wextra \
	-flto -fanalyzer \
	-Wno-unused-parameter \
	-march=rv32imac_zicsr -mabi=ilp32 \
	-Os -g3 \
	-fdata-sections -ffunction-sections -Wl,--gc-sections \
	-nostartfiles \
	-T $(LINK_SCRIPT) \
	-DINT_SOFT

all: patch $(FW_NAME).elf $(FW_NAME).bin $(FW_NAME).asm
	$(SZ) $(FW_NAME).elf

flash: all
	wchisp flash $(FW_NAME).bin

clean:
	rm -f *.out $(FW_NAME).asm $(FW_NAME).elf $(FW_NAME).bin

patch:
	sed -i -e 's/__attribute__((interrupt("WCH-Interrupt-fast")))/__INTERRUPT/g' \
		$(CH58X_SDK)/../FreeRTOS/FreeRTOS/portable/GCC/RISC-V/port.c
	sed -i -e 's/void FLASH_ROM_READ(UINT32 StartAddr, PVOID Buffer, UINT32 len);//g' \
		$(CH58X_SDK)/StdPeriphDriver/inc/CH58x_flash.h

format:
	clang-format -i *.c *.h

$(FW_NAME).elf:
	$(CC) $(CFLAGS) $(INCS) $(SRCS) $(LIBS) -o $(FW_NAME).elf

$(FW_NAME).asm: $(FW_NAME).elf
	$(OD) -S -D $(FW_NAME).elf > $(FW_NAME).asm

$(FW_NAME).bin: $(FW_NAME).elf
	$(OC) -O binary $(FW_NAME).elf $(FW_NAME).bin
