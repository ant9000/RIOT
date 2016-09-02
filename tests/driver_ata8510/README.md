# About
This is a manual test application for the ATA8510 radio driver

For running this test, you need to connect/configure the following pins of your
radio device:
- SPI MISO
- SPI MOSI
- SPI CLK
- CS (chip select)
- RESET
- SLEEP
- INT (external interrupt)

# Usage
For testing the radio driver you can use the netif and txtsnd shell commands
that are included in this application.

To build a receiver:
    BOARD=yarm make flash

To build a transmitter (default ID is 1):
    BOARD=yarm TX=1 make flash

To build a transmitter with a different ID (range is 1-9):
    BOARD=yarm TX=1 ID8510=5 make flash

