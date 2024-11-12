# pip install esptool esp-coredump
esptool.py --p /dev/ttyUSB0  read_flash 0x3F0000 0x10000 core.bin
esp-coredump info_corefile --core  core.bin --core-format raw  xxx.elf