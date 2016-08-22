APPLICATION = driver_ata8510
include ../Makefile.tests_common

FEATURES_REQUIRED = periph_spi periph_gpio

DISABLE_MODULE += auto_init

USEMODULE += od
USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps

USEMODULE += ata8510

# ROB use xtimer
USEMODULE += xtimer

# ROB use rtc
FEATURES_OPTIONAL += periph_rtc

# ROB use random
USEMODULE += random

# ROB use checksum
USEMODULE += checksum

CFLAGS += -DDEVELHELP

include $(RIOTBASE)/Makefile.include
