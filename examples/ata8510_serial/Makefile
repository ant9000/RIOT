APPLICATION = ata8510_serial
BOARD ?= yarm
RIOTBASE ?= $(CURDIR)/../..
QUIET ?= 1

FEATURES_REQUIRED = periph_spi periph_gpio periph_i2c

DISABLE_MODULE += auto_init

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

USEMODULE += ata8510
USEMODULE += daisy24
USEMODULE += xtimer

CFLAGS += -DDEVELHELP

include $(RIOTBASE)/Makefile.include
