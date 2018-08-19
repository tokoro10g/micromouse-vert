SHELL = /bin/sh
TARGET_ARCH   = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
INCLUDE_DIRS  = -I autogen/Drivers/STM32F4xx_HAL_Driver/Inc \
				-I autogen/Drivers/CMSIS/Include \
				-I autogen/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
				-I ../libmazesolver \
				-I ../libtrajectory \
				-I ../mymath \
				-I system \
				-I .
FIRMWARE_OPTS = -DUSE_HAL_DRIVER -DSTM32F411xE -DARM_MATH_CM4
COMPILE_OPTS  = -MD -std=gnu++11 -O3 -g3 -ffunction-sections -fpermissive -fdata-sections -fsigned-char -fno-rtti -fno-exceptions -Wall -Wno-write-strings -Wno-unused-but-set-variable -fmessage-length=0 -pipe -fno-threadsafe-statics -flto $(INCLUDE_DIRS) $(BOARD_OPTS) $(FIRMWARE_OPTS)
TARGETS = $(patsubst %.c,%.o,$(wildcard *.c)) \
		  $(patsubst %.cpp,%.o,$(wildcard *.cpp)) \
		  $(patsubst %.cpp,%.o,$(wildcard ../mymath/*.cpp)) \
		  $(patsubst %.cpp,%.o,$(wildcard ../libtrajectory/*.cpp)) \
		  $(patsubst %.cpp,%.o,$(wildcard ../libmazesolver/*.cpp)) \
		  $(patsubst %.cpp,%.o,$(wildcard machine/*.cpp)) \
		  $(patsubst %.cpp,%.o,$(wildcard control/*.cpp)) \
		  $(patsubst %.c,%.o,$(wildcard autogen/Drivers/STM32F4xx_HAL_Driver/Src/*.c)) \
		  system/startup_stm32f411xe.o \
		  system/system_stm32f4xx.o \
		  system/stm32f4xx_hal_msp.o \
		  system/LibraryHacks.o
DEPS = $(patsubst %.o,%.d,$(TARGETS))

#SUBDIRS = ../libmazesolver

CC      = arm-none-eabi-g++
CXX     = $(CC)
AS      = $(CC)
LD      = $(CC)
AR      = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
CFLAGS  = $(COMPILE_OPTS)
CXXFLAGS= $(COMPILE_OPTS) -include stdint.h
ASFLAGS = -x assembler-with-cpp -c $(TARGET_ARCH) $(COMPILE_OPTS) 
LDFLAGS = -ggdb3 -Wl,-wrap,__aeabi_unwind_cpp_pr0 -Wl,-wrap,__aeabi_unwind_cpp_pr1 -Wl,-wrap,__aeabi_unwind_cpp_pr2 -Wl,--gc-sections,-Map=bin/main.map,-cref -specs=nano.specs -u _printf_float -T system/STM32F411CEUx_FLASH.ld $(INCLUDE_DIRS)

.PHONY: all

all: bin/main.hex

#$(SUBDIRS):
#	$(MAKE) -f Makefile_arm_hal -C $@

bin/main.hex: $(TARGETS)
	$(LD) $(LDFLAGS) $(TARGET_ARCH) $^ -o bin/main.elf 
	$(OBJCOPY) -O ihex bin/main.elf bin/main.hex

clean:
	rm -rf *.o *.s bin/* system/*.o machine/*.o control/*.o ../libmazesolver/*.o autogen/Drivers/STM32F4xx_HAL_Driver/Src/*.o ../libmazesolver/*.o ../libmazesolver/libmazesolver_arm_hal.a
	rm -rf *.d *.s system/*.d machine/*.d control/*.d ../libmazesolver/*.d autogen/Drivers/STM32F4xx_HAL_Driver/Src/*.d ../libmazesolver/*.d

-include $(DEPS)
