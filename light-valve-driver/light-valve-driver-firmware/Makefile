MAKE=make

CUBEMX_DIR=./thirdparty/cubemx-light-valve-driver
CUBEMX_MAKEFILE=$(CUBEMX_DIR)/Makefile

# Can't use 'app' as default target name since that's cubemx's
default: app_default

# Include generated makefile just after our default target override so it establishes all of the variables
# needed to compile / link the CubeMx source files
include $(CUBEMX_MAKEFILE)

# Re-assign OBJECTS to CUBEMX_OBJECTS and prefix with path to cubemx build dir from here
CUBEMX_UNALTERED_OBJECTS := $(OBJECTS)
CUBEMX_OBJECTS := $(addprefix $(CUBEMX_DIR)/,$(CUBEMX_UNALTERED_OBJECTS))
CUBEMX_C_SOURCES := $(C_SOURCES)
CUBEMX_C_SOURCES := $(addprefix $(CUBEMX_DIR)/,$(CUBEMX_C_SOURCES))

LDSCRIPT := $(addprefix $(CUBEMX_DIR)/,$(LDSCRIPT))

# Save original build dir relative to cubemx dir to use for cubemx specific target
CUBEMX_BUILD_DIR := $(BUILD_DIR)

APP_BUILD_DIR=out/

# This is a bit of a hack. We're overriding BUILD_DIR used in the cubemx makefile so the invocation of
# `-Map=$(BUILD_DIR)/$(TARGET).map` in the assignment of LDFLAGS works. Ideally ST would use a separate
# variable for the map file name that we could use our prefix trick on, but this is the best way I could
# think of for this to work
BUILD_DIR := $(APP_BUILD_DIR)

APP_INCLUDES = -Iinc
APP_C_SOURCES = src/app_main.c
APP_ASM_SOURCES = 

# Create correctly prefixed object list for application source files
APP_OBJECTS = $(addprefix $(APP_BUILD_DIR)/,$(notdir $(APP_C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
APP_OBJECTS += $(addprefix $(APP_BUILD_DIR)/,$(notdir $(APP_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Bundle app-specific and cubemx variables for use in our compilation targets
OBJECTS := $(CUBEMX_OBJECTS) $(APP_OBJECTS)
INCLUDES := $(CUBEMX_C_SOURCES) $(APP_INCLUDES)
C_SOURCES := $(CUBEMX_C_SOURCES) $(APP_C_SOURCES)

# 'Actual' default target referenced by dummy default target listed before cubemx makefile is included
app_default: $(APP_BUILD_DIR)/$(TARGET).elf

# Override the 'build/*.o' target for all the cubemx files. I would have thought this would correctly know to use the
# targets that match in the cubemx file, but even without the BUILD_DIR override hack above for the map file it can't
# find them. This just executes the cubemx makefile's make target for each object file.
$(CUBEMX_BUILD_DIR)/%.o:
	@$(MAKE) -C $(CUBEMX_DIR) $@

# Build app object files from app C source files
$(APP_BUILD_DIR)/%.o: %.c Makefile | $(APP_BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(APP_BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

# Build app object files from app assembly source files
$(APP_BUILD_DIR)/%.o: %.s Makefile | $(APP_BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

# Build the elf !
$(APP_BUILD_DIR)/$(TARGET).elf: $(APP_OBJECTS) $(CUBEMX_UNALTERED_OBJECTS)  Makefile
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
