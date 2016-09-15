The driver works only if the ATA8510 EEPROM is configured with the provided
firmware. To flash it, use avrdude like this:

"""
avrdude -c atmelice_isp -p ata8510 -C +ata8510.conf -U eeprom:w:firmware.hex:i
"""

Tested with:
- Debian Jessie avrdude 6.1-2
- avrdude 6.3 (from http://www.nongnu.org/avrdude/)
