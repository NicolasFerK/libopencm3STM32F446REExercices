BOOTLOADER_SIZE = 0x08000
BOOTLOADER_FILE = r'C:\Users\nicol\Documents\PlatformIO\Projects\bootloader-libopencm3\.pio\build\genericSTM32F446RE\firmware.bin'

with open(BOOTLOADER_FILE, "rb") as f:
    raw_file = f.read()

bytes_to_pad = BOOTLOADER_SIZE - len(raw_file)
padding = bytes([0xff for _ in range(bytes_to_pad)])

with open(BOOTLOADER_FILE, "wb") as f:
    f.write(raw_file + padding)