# make RC_MODEL=NOTANK LED_MODE=LED_MODE_74HC595D TX_TYPE=RADIOMASTER BOARD=nano
#
# For flashing:
# Nano, old bootloader: make PORT=/dev/ttyUSB0 BAUD=57600 flash
# Nano, new bootloader: make PORT=/dev/ttyUSB0 BAUD=115200 flash
# Nano Every:           make PORT=/dev/ttyUSB0 BOARD=nano-every flash
#
# All overrides (RC_MODEL, LED_MODE, TX_TYPE) are injected via -D flags - so
# make sure to guard new defines in the .ino with #ifdef when they should also
# be customisable.

# ---------------------------------------------------------------------------
# User-configurable options
# ---------------------------------------------------------------------------
RC_MODEL  ?= NOTANK
LED_MODE  ?= LED_MODE_74HC595D
TX_TYPE   ?= RADIOMASTER
BOARD     ?= nano
# Nano bootloader baud: 57600 (old / most clones) or 115200
BAUD      ?= 57600

# arduino-cli user directory. Override if your sketchbook lives elsewhere.
# This is mostly chosen for compatibility with the Arduino IDE
ARDUINO_USER_DIR ?= $(HOME)/sketchbook
export ARDUINO_DIRECTORIES_USER := $(ARDUINO_USER_DIR)

# ---------------------------------------------------------------------------
# Board presets
# ---------------------------------------------------------------------------
ifeq ($(BOARD),nano)
  # The compiled hex is identical for old and new bootloaders (same ATmega328p).
  # We default to the old FQBN for compatibility; any ATmega328p Nano FQBN works.
  FQBN         = arduino:avr:nano:cpu=atmega328old
  MCU          = atmega328p
  AVRDUDE_PROG = arduino
else ifeq ($(BOARD),nano-every)
  FQBN         = arduino:megaavr:nano
  MCU          = atmega4809
  AVRDUDE_PROG = jtag2updi
  BAUD         = 115200
else
  $(error Unknown BOARD='$(BOARD)'. Valid: nano, nano-every)
endif

SKETCH       = rcfw.ino
VARIANT      = $(RC_MODEL)_$(LED_MODE)_$(TX_TYPE)_$(BOARD)
BUILD_DIR    = build/$(VARIANT)
FIRMWARE_DIR = firmware
HEX_SRC      = $(BUILD_DIR)/$(SKETCH).hex
HEX_OUT      = $(FIRMWARE_DIR)/rcfw_$(VARIANT).hex

# Pass all user defines into the Arduino build
EXTRA_FLAGS  = -DRC_MODEL=$(RC_MODEL) -DLED_MODE=$(LED_MODE) -DTX_TYPE=$(TX_TYPE)

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------
.PHONY: all build flash upload clean monitor deps verify all-variants

all: build

build: verify
	@echo "==> Building $(VARIANT)"
	@mkdir -p $(FIRMWARE_DIR)
	arduino-cli compile --fqbn $(FQBN) \
	  --build-property "build.extra_flags=$(EXTRA_FLAGS)" \
	  --output-dir $(BUILD_DIR) \
	  $(SKETCH)
	@cp $(HEX_SRC) $(HEX_OUT)
	@echo "==> $(HEX_OUT)"

flash: build
	@if [ -z "$(PORT)" ]; then \
	  echo "PORT not set. Example: make PORT=/dev/ttyUSB0 flash"; exit 1; fi
	@echo "==> Flashing $(MCU) on $(PORT) via $(AVRDUDE_PROG) at $(BAUD) baud"
	avrdude -p $(MCU) -c $(AVRDUDE_PROG) -P $(PORT) -b $(BAUD) \
	  -U flash:w:$(HEX_OUT):i

upload: build
	arduino-cli upload -p $(PORT) --fqbn $(FQBN) $(SKETCH)

clean:
	rm -rf build $(FIRMWARE_DIR)

monitor:
	@which minicom >/dev/null 2>&1 && minicom -D $(PORT) -b 115200 || \
	  screen $(PORT) 115200

verify:
	@which arduino-cli >/dev/null || (echo "arduino-cli not found. https://arduino.github.io/arduino-cli/installation/"; exit 1)
	@which avrdude   >/dev/null || (echo "avrdude not found. Install via your package manager."; exit 1)

# Install Arduino cores and libraries.  Run once after cloning.
deps:
	arduino-cli core update-index
	arduino-cli core install arduino:avr
	arduino-cli core install arduino:megaavr
	arduino-cli lib install Servo
	arduino-cli lib install IBusBM
	@echo ""
	@echo "AlfredoCRSF is not in the Arduino Library Manager."
	@echo "If you need CRSF support, install it manually:"
	@echo "  arduino-cli lib install --git-url https://github.com/AlfredoSystems/AlfredoCRSF.git"

# ---------------------------------------------------------------------------
# CI target: build every supported combination
# ---------------------------------------------------------------------------
ALL_MODELS = SNOW_MOBILE RC_BENCHY NOTANK
ALL_MODES  = LED_MODE_DIRECT LED_MODE_74HC595D
ALL_TX     = FLYSKY_ST RADIOMASTER

all-variants:
	@mkdir -p $(FIRMWARE_DIR)
	@for model in $(ALL_MODELS); do \
	  for mode in $(ALL_MODES); do \
	    for tx in $(ALL_TX); do \
	      echo ""; \
	      echo ">>> $$model / $$mode / $$tx / $(BOARD)"; \
	      $(MAKE) RC_MODEL=$$model LED_MODE=$$mode TX_TYPE=$$tx BOARD=$(BOARD) build || exit 1; \
	    done; \
	  done; \
	done
	@echo ""
	@echo "All variants ready in $(FIRMWARE_DIR)/"
