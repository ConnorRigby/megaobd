ifeq ($(OS),Windows_NT)
	ARDUINO_PATH = $(shell "sysutils/arduino_path.cmd")
	MKDIR = mkdir $(subst /,\,$(1)) > nul 2>&1 || (exit 0)
	RM = $(wordlist 2,65535,$(foreach FILE,$(subst /,\,$(1)),& del /f /q $(FILE) > nul 2>&1)) || (exit 0)
	RMDIR = rmdir $(subst /,\,$(1)) > nul 2>&1 || (exit 0)
	ECHO = @echo $(1)
else
	ARDUINO_PATH = $(shell "sysutils/arduino_path.sh")
	MKDIR = mkdir -p $(1)
	RM = rm -f $(1) > /dev/null 2>&1 || true
	RMDIR = rmdir $(1) > /dev/null 2>&1 || true
	ECHO = @echo $(1)
endif

ARDUINO_PATH_UNQUOTED = $(subst ",,$(ARDUINO_PATH))

ifeq ($(ARDUINO_PATH),)
	$(error Arduino path not found)
endif

ARDUINO_PROJECT := src.ino

ARDUINO_BUILDER = "$(ARDUINO_PATH_UNQUOTED)/arduino-builder"
ARDUINO_HARDWARE := "$(ARDUINO_PATH_UNQUOTED)/hardware"
ARDUINO_TOOLS_BUILDER := "$(ARDUINO_PATH_UNQUOTED)/tools-builder"
ARDUINO_TOOLS_HARDWARE_AVR := "$(ARDUINO_PATH_UNQUOTED)/hardware/tools/avr"
ARDUINO_LIBRARIES := "$(ARDUINO_PATH_UNQUOTED)/libraries"

BUILD_DIR := "$(CURDIR)/build"
BUILD_LIBRARIES := "$(CURDIR)/libraries"

FQBN="arduino:avr:uno"

all: build/$(ARDUINO_PROJECT).hex

build/src.ino.hex: src/src.ino
	$(call MKDIR,build libraries)
	$(ARDUINO_BUILDER) -compile -logger=machine -hardware $(ARDUINO_HARDWARE) -tools $(ARDUINO_TOOLS_BUILDER) -tools $(ARDUINO_TOOLS_HARDWARE_AVR) -built-in-libraries $(ARDUINO_LIBRARIES) -libraries $(BUILD_LIBRARIES) -fqbn=$(FQBN) -ide-version=10607 -build-path $(BUILD_DIR) -warnings=none -prefs=build.warn_data_percentage=80 -verbose "$(CURDIR)/src/src.ino"

clean:
	$(call RM,build/core/* build/libraries/EEPROM/* build/libraries/* build/preproc/* build/sketch/* build/* libraries/*)
	$(call RMDIR,build/core build/libraries/EEPROM build/libraries build/preproc build/sketch build libraries)

test:
	$(call ECHO,$(ARDUINO_BUILDER))
	$(call ECHO,$(ARDUINO_PATH))
