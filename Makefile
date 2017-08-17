EXECUTABLE=out.elf
STM32_LIBS=/opt/STM32Cube_FW_L0_V1.8.0

CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

DEFS = -DSTM32L053xx
MCU = cortex-m0plus
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian
STD = -std=gnu11

STM32_INCLUDES = -I$(STM32_LIBS)/Utilities \
	-I$(STM32_LIBS)/Drivers/CMSIS/Device/ST/STM32L0xx/Include \
	-I$(STM32_LIBS)/Drivers/CMSIS/Include \
	-I$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Inc \
	-I$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/inc\

OPTIMIZE = -O0

SEMIHOST = --specs=rdimon.specs -lc -lrdimon

CFLAGS	= $(MCFLAGS) $(STD) -g $(OPTIMIZE)  $(DEFS) -Iinc -I./ -I./ $(STM32_INCLUDES) $(SEMIHOST) -Wl,-T,src/sys/STM32L053C8Tx_FLASH.ld

AFLAGS	= $(MCFLAGS)

SRC = \
	src/main.c \
	src/tsl_user.c \
	src/sys/startup_stm32l053xx.s \
	src/sys/stm32l0xx_hal_msp.c \
	src/sys/stmCriticalSection.c \
	src/sys/stm32l0xx_it.c \
	src/sys/system_stm32l0xx.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_acq.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_time.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_dxs.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_linrot.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_filter.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_touchkey.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_acq_tsc.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_globals.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_ecs.c \
	$(STM32_LIBS)/Middlewares/ST/STM32_TouchSensing_Library/src/tsl_object.c \
	$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
	$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
	$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
	$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
	$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_tsc.c \
	$(STM32_LIBS)/Drivers/STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \


OBJDIR = .
OBJ = $(SRC:%.c=$(OBJDIR)/%.o)

all: $(EXECUTABLE)

$(EXECUTABLE): $(SRC)
	$(CC) $(CFLAGS) $^ -o $@ -lm

debug_server:
	sudo openocd -f board/stm32l0discovery.cfg

debug_gdb:
	arm-none-eabi-gdb out.elf -ex "target remote localhost:3333" -ex "monitor reset halt" -ex "load" -ex "monitor reset halt" -ex "monitor arm semihosting enable" -ex "b main" -ex "c"
