# STM32 firmware Makefile for the AGV
#
# Usage:
#   make              → build firmware.elf + firmware.bin
#   make flash        → upload via st-flash (ST-Link V2)
#   make flash-openocd→ upload via openocd (any debugger)
#   make clean        → remove build artefacts
#
# Configure CUBE_DIR and CHIP/BOARD below to match your hardware.

# ── User config ─────────────────────────────────────────────────────
# Path to the cloned STM32CubeF4 (or F1/F3/...) repository.
CUBE_DIR  := $(HOME)/STM32CubeF4

# Chip family (used to find Drivers/Inc).
CHIP_FAMILY := STM32F4xx

# Which HAL source files to include. Adjust to your chip's flash/RAM size.
DEFS := -DSTM32F411xE
# Other options: -DSTM32F407xx -DSTM32F103xB -DSTM32F302xC ...

# Toolchain
CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

# ── HAL sources ─────────────────────────────────────────────────────
HAL_DIR := $(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR := $(CUBE_DIR)/Drivers/CMSIS

INCLUDES := \
  -I. \
  -I$(HAL_DIR)/Inc \
  -I$(HAL_DIR)/Inc/Legacy \
  -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include \
  -I$(CMSIS_DIR)/Include

CFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          -O2 -g3 -Wall -ffunction-sections -fdata-sections \
          $(DEFS) $(INCLUDES)

# NOTE: ST removed the linker scripts from the upstream Drivers/CMSIS tree.
# The example project at Projects/STM32F411RE-Nucleo/Templates/STM32CubeIDE/
# is the canonical place to find them now.
LDSCRIPT := $(CUBE_DIR)/Projects/STM32F411RE-Nucleo/Templates/STM32CubeIDE/STM32F411RETX_FLASH.ld
# Other chips: STM32F407VGTx_FLASH.ld, STM32F103C8Tx_FLASH.ld, etc.

LDFLAGS := -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
           -specs=nano.specs -T$(LDSCRIPT) -Wl,--gc-sections \
           -Wl,-Map=firmware.map,--cref

# ── Sources ─────────────────────────────────────────────────────────
SRCS := main_v3.c \
        stm32f4xx_hal_msp.c \
        $(CMSIS_DIR)/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
        $(HAL_DIR)/Src/stm32f4xx_hal.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_uart.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_tim.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_tim_ex.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_gpio.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_rcc.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_cortex.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_dma.c \
        $(HAL_DIR)/Src/stm32f4xx_hal_pwr.c

STARTUP := $(CMSIS_DIR)/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f411xe.s

# ── Build rules ─────────────────────────────────────────────────────
BUILD := build
OBJS  := $(patsubst %.c,$(BUILD)/%.o,$(SRCS)) $(patsubst %.s,$(BUILD)/%.o,$(STARTUP))
ELF   := firmware.elf
BIN   := firmware.bin

.PHONY: all flash flash-openocd clean

all: $(BIN)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) -mcpu=cortex-m4 -mthumb -c $< -o $@

$(ELF): $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

flash: $(BIN)
	st-flash --connect-under-reset write $(BIN) 0x08000000

flash-openocd: $(ELF)
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	        -c "program $(ELF) verify reset exit"

clean:
	rm -rf $(BUILD) $(ELF) $(BIN) firmware.map