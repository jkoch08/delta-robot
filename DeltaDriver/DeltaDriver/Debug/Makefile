################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../DeltaDriver.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
DeltaDriver.o

OBJS_AS_ARGS +=  \
DeltaDriver.o

C_DEPS +=  \
DeltaDriver.d

C_DEPS_AS_ARGS +=  \
DeltaDriver.d

OUTPUT_FILE_PATH +=DeltaDriver.elf

OUTPUT_FILE_PATH_AS_ARGS +=DeltaDriver.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+=  \
J:/Dropbox/CAD/Me115b\ Delta\ Robot/Code/CM700_Library/lib/libdynamixel.a \
J:/Dropbox/CAD/Me115b\ Delta\ Robot/Code/CM700_Library/lib/libserial.a \
J:/Dropbox/CAD/Me115b\ Delta\ Robot/Code/CM700_Library/lib/libzigbee.a

# AVR32/GNU C Compiler



./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"C:\WinAVR-20100110\bin" -I"C:\WinAVR-20100110\utils\bin" -I"../../../CM700_Library/include"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -mrelax -g2 -Wall -mmcu=atmega2561 -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="DeltaDriver.map" -Wl,--start-group -Wl,-lm -Wl,-ldynamixel -Wl,-lserial -Wl,-lzigbee  -Wl,--end-group -Wl,-L"J:\Dropbox\CAD\Me115b Delta Robot\Code\CM700_Library\lib"  -Wl,--gc-sections -mrelax -mmcu=atmega2561 -Wl,-u,vfprintf -lprintf_flt  
	@echo Finished building target: $@
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "DeltaDriver.elf" "DeltaDriver.hex"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "DeltaDriver.elf" "DeltaDriver.eep" || exit 0
	"C:\WinAVR-20100110\bin\avr-objdump.exe" -h -S "DeltaDriver.elf" > "DeltaDriver.lss"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "DeltaDriver.elf" "DeltaDriver.srec"
	"C:\WinAVR-20100110\bin\avr-size.exe" "DeltaDriver.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "DeltaDriver.elf" "DeltaDriver.a" "DeltaDriver.hex" "DeltaDriver.lss" "DeltaDriver.eep" "DeltaDriver.map" "DeltaDriver.srec" "DeltaDriver.usersignatures"
	