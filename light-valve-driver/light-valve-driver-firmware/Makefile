MAKE=make

THIRDPARTY_DIR=./thirdparty
CUBEMX_DIR=$(THIRDPARTY_DIR)/cubemx-light-valve-driver
CUBEMX_MAKEFILE=$(CUBEMX_DIR)/Makefile
FREERTOS_CLI_DIR=$(THIRDPARTY_DIR)/FreeRTOS-Plus-CLI

# Can't use 'app' as default target name since that's cubemx's
default: app_default

# Include generated makefile just after our default target override so it establishes all of the variables
# needed to compile / link the CubeMx source files
include $(CUBEMX_MAKEFILE)

# Re-assign OBJECTS to CUBEMX_OBJECTS and prefix with path to cubemx build dir from here
CUBEMX_UNALTERED_OBJECTS := $(OBJECTS)
CUBEMX_OBJECTS := $(addprefix $(CUBEMX_DIR)/,$(CUBEMX_UNALTERED_OBJECTS))
CUBEMX_C_INCLUDES := $(C_INCLUDES)
CUBEMX_C_INCLUDES := $(CUBEMX_C_INCLUDES:-I%=%) # Strip leading '-I's out so we can stick path prefix in
CUBEMX_C_INCLUDES := $(addprefix -I$(CUBEMX_DIR)/,$(CUBEMX_C_INCLUDES)) # Add '-I' and cubemx thirdparty path back in
CUBEMX_C_SOURCES := $(C_SOURCES)
CUBEMX_C_SOURCES := $(addprefix $(CUBEMX_DIR)/,$(CUBEMX_C_SOURCES))
LDSCRIPT := $(addprefix $(CUBEMX_DIR)/,$(LDSCRIPT))
FREERTOS_CLI_C_INCLUDES := -I$(FREERTOS_CLI_DIR)
FREERTOS_CLI_C_SOURCES := $(addprefix $(FREERTOS_CLI_DIR)/,FreeRTOS_CLI.c)

# Save original build dir relative to cubemx dir to use for cubemx specific target
CUBEMX_BUILD_DIR := $(BUILD_DIR)

APP_BUILD_DIR=out

# This is a bit of a hack. We're overriding BUILD_DIR used in the cubemx makefile so the invocation of
# `-Map=$(BUILD_DIR)/$(TARGET).map` in the assignment of LDFLAGS works. Ideally ST would use a separate
# variable for the map file name that we could use our prefix trick on, but this is the best way I could
# think of for this to work
BUILD_DIR := $(APP_BUILD_DIR)

APP_INCLUDES = -Iinc

APP_C_SOURCES = \
src/app_main.c \
src/gpio.c \
src/heartbeat_task.c \
src/cli_task.c \
src/uart.c \
src/cli_commands.c


# TODO this needs to be extended to cycle through all thirdparty dirs
THIRDPARTY_OBJECTS += $(addprefix $(APP_BUILD_DIR)/,$(notdir $(FREERTOS_CLI_C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(FREERTOS_CLI_C_SOURCES)))

APP_ASM_SOURCES = 

# Create correctly prefixed object list for application source files
APP_OBJECTS = $(addprefix $(APP_BUILD_DIR)/,$(notdir $(APP_C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
APP_OBJECTS += $(addprefix $(APP_BUILD_DIR)/,$(notdir $(APP_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Bundle app-specific and cubemx variables for use in our compilation targets
OBJECTS := $(CUBEMX_OBJECTS) $(APP_OBJECTS) $(THIRDPARTY_OBJECTS)
INCLUDES := $(CUBEMX_C_INCLUDES) $(APP_INCLUDES) $(FREERTOS_CLI_C_INCLUDES)
C_SOURCES := $(CUBEMX_C_SOURCES) $(APP_C_SOURCES) $(FREERTOS_CLI_C_SOURCES)
APP_CFLAGS := $(MCU) $(C_DEFS) $(INCLUDES) $(OPT) -Wall -Werror -fdata-sections -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)"
ifeq ($(DEBUG), 1)
APP_CFLAGS += -g -gdwarf-2
endif

# 'Actual' default target referenced by dummy default target listed before cubemx makefile is included
app_default: $(APP_BUILD_DIR)/$(TARGET).elf

$(APP_BUILD_DIR)/FreeRTOS_CLI.o: $(FREERTOS_CLI_DIR)/FreeRTOS_CLI.c Makefile | $(APP_BUILD_DIR) 
	$(CC) -c $(APP_CFLAGS) -Wa,-a,-ad,-alms=$(APP_BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Override build dir dependency from our makefile's directory because main.o dependency below tries to make cubemx 'build' dir in
# top level instead of within cubemx dir when we override it.
$(CUBEMX_DIR)/$(CUBEMX_BUILD_DIR):
	$(MAKE) -C $(CUBEMX_DIR) $(CUBEMX_BUILD_DIR)

# Override SPECIFICALLY the main.o target to compile cubemx's main.c This is so we can force it to compile with one extra -I include
# flag of ../../inc. This is necessary so it can include our app_main.h so it can correctly call the app_main() function. All other
# cubemx *.o targets handled in override below which just passes call through to matching cubemx makefile target.
$(CUBEMX_BUILD_DIR)/main.o: $(CUBEMX_DIR)/Src/main.c $(CUBEMX_DIR)/Makefile | $(CUBEMX_DIR)/$(CUBEMX_BUILD_DIR)
	@cd $(CUBEMX_DIR); \
	$(CC) -c $(CFLAGS) -I../../inc -Wa,-a,-ad,-alms=$(CUBEMX_BUILD_DIR)/$(notdir $(<:.c=.lst)) Src/main.c -o $@

# Override the 'build/*.o' target for all the cubemx files that aren't main.o to simply call the cubemx makefile target for them.
# I would have thought this would correctly know to use the targets that match in the cubemx file, but even without the BUILD_DIR
# override hack above for the map file it can't find them.
$(CUBEMX_BUILD_DIR)/%.o:
	@$(MAKE) -C $(CUBEMX_DIR) $@

# Build app object files from app C source files
$(APP_BUILD_DIR)/%.o: %.c Makefile | $(APP_BUILD_DIR) 
	$(CC) -c $(APP_CFLAGS) -Wa,-a,-ad,-alms=$(APP_BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Build app object files from app assembly source files
$(APP_BUILD_DIR)/%.o: %.s Makefile | $(APP_BUILD_DIR)
	$(AS) -c $(APP_CFLAGS) $< -o $@

# Build the elf !
$(APP_BUILD_DIR)/$(TARGET).elf: $(APP_OBJECTS) $(CUBEMX_UNALTERED_OBJECTS) $(THIRDPARTY_OBJECTS)  Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

# Also objcopy a hex from the elf
$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

# Generate build dir if doesn't exist
$(APP_BUILD_DIR):
	mkdir $@

# I don't know how to get this to be called 'clean' without the error for overriding the cubemx makefile one
.PHONY: appclean
appclean:
ifdef APP_BUILD_DIR
	@rm -rf $(APP_BUILD_DIR)
endif
ifdef CUBEMX_BUILD_DIR
	@rm -rf $(CUBEMX_DIR)/$(CUBEMX_BUILD_DIR) # Can't just call cubemx clean target because of the BUILD_DIR override hack for the map file :/
endif
