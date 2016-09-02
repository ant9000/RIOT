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

RX ?= 1
TX ?= 0
ifeq ($(TX),1)
  RX = 0
  ID8510 ?= 1
  CFLAGS += -DTHREADTXRAND -DID8510=$(ID8510)
endif
ifeq ($(RX),1)
  CFLAGS += -DTHREADCHECKRXERRORS
endif

include $(RIOTBASE)/Makefile.include
