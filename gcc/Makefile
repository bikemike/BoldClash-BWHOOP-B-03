TARGET=bwhoop
EXECUTABLE=$(TARGET).elf

TARGET_DBG=$(TARGET)_dbg
EXECUTABLE_DBG=$(TARGET_DBG).elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
DB=arm-none-eabi-gdb

topdir = ..

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F031
STARTUP = $(topdir)/Libraries/CMSIS/Device/ST/STM32F0xx/Source/Templates/gcc_ride7/startup_stm32f030.s

MCU = cortex-m0
MCFLAGS = -mcpu=$(MCU) -g -ggdb -mthumb -fdata-sections -ffunction-sections -fsingle-precision-constant -ffast-math -nostartfiles   --specs=nano.specs -flto
MCFLAGS_DBG = -mcpu=$(MCU) -g -ggdb -mthumb -fdata-sections -ffunction-sections -fsingle-precision-constant -ffast-math -nostartfiles   --specs=rdimon.specs -lrdimon -u _printf_float -flto

INCLUDES = -I$(topdir)/Silverware/src/ \
	-I$(topdir)/Libraries/CMSIS/Device/ST/STM32F0xx/Include/ \
	-I$(topdir)/Libraries/CMSIS/Include/ \
	-I$(topdir)/Utilities/ \
	-I$(topdir)/Libraries/STM32F0xx_StdPeriph_Driver/inc/

OPTIMIZE = -Os
OPTIMIZE_DBG = -Og

CFLAGS = $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(INCLUDES)  -Wl,-T,flash.ld,-Map,output.map,--gc-sections  -std=gnu99
CFLAGS_DBG = $(MCFLAGS_DBG)  $(OPTIMIZE_DBG)  $(DEFS) -I. -I./ $(INCLUDES)  -Wl,-T,flash.ld,-Map,output.map,--gc-sections  -std=gnu99

#AFLAGS = $(MCFLAGS)

SRC = $(wildcard $(topdir)/Silverware/src/*.c) \
	$(wildcard $(topdir)/Silverware/src/*.h) \
	$(wildcard $(topdir)/Silverware/src/*.cpp) \
	$(topdir)/Libraries/STM32F0xx_StdPeriph_Driver/src/*.c  \
	$(topdir)/Utilities/system_stm32f0xx.c \
	methods.c

OBJDIR = .
OBJ = $(patsubst %.c,$(OBJDIR)/%.o,$(filter %.c,$(SRC)))
OBJ += $(patsubst %.cpp,$(OBJDIR)/%.o,$(filter %.cpp,$(SRC)))
OBJ += Startup.o


all: $(TARGET)

$(TARGET).hex: $(EXECUTABLE)
	$(CP) -O ihex $^ $@
$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -o $@
	arm-none-eabi-size $(EXECUTABLE)
	
debug: $(TARGET_DBG)

$(TARGET_DBG).hex: $(EXECUTABLE_DBG)
	$(CP) -O ihex $^ $@
$(TARGET_DBG): $(EXECUTABLE_DBG)
	$(CP) -O binary $^ $@

$(EXECUTABLE_DBG): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS_DBG) $^ -lm -o $@
	arm-none-eabi-size $(EXECUTABLE_DBG)

clean:
	rm -f Startup.lst $(TARGET) $(TARGET).lst $(OBJ) $(AUTOGEN) \
		$(TARGET).out $(TARGET).hex  $(TARGET).map \
		$(TARGET).dmp $(EXECUTABLE)
	rm -f Startup.lst $(TARGET_DBG) $(TARGET_DBG).lst $(OBJ) $(AUTOGEN) \
		$(TARGET_DBG).out $(TARGET_DBG).hex  $(TARGET_DBG).map \
		$(TARGET_DBG).dmp $(EXECUTABLE_DBG)

erase:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -c init -c "reset halt" -c "stm32f0x unlock 0" -c "reset run" -c shutdown

flash:
	openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f0x.cfg -c init -c "reset halt" -c "flash write_image erase $(TARGET) 0x08000000" -c "verify_image $(TARGET) 0x08000000" -c "reset run" -c shutdown

flash-bm: $(TARGET).elf
	$(DB) $(TARGET).elf -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

flash-debug-bm: $(TARGET_DBG).elf
	$(DB) $(TARGET_DBG).elf -ex "set confirm off" -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "detach" -ex "quit"

debug-bm: $(TARGET_DBG).elf
	$(DB) $(TARGET_DBG).elf -ex "target extended-remote /dev/ttyACM0" -ex "monitor swdp_scan" -ex "attach 1"
