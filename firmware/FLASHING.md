# Flashing the AGV STM32 — options

You have three working paths. Pick the one your hardware lets you do.

## Files to copy off the Jetson

Everything you need is in `firmware/`:

| File           | Size      | MD5                              | What it is              |
|----------------|-----------|----------------------------------|-------------------------|
| `firmware.hex` | 22 103 B  | `ec4d4998fc14ab6d9e5342ea1166f3cc` | Intel HEX (try this first) |
| `firmware.bin` |  7 828 B  | `593ea50f75576b2c985b5fd7a4ef091b` | raw binary              |
| `main_v3.c`    | 15 858 B  |                                  | source — only if you have arm-none-eabi-gcc on the other laptop |
| `firmware.elf` | 1 MB      |                                  | debug symbols — not for flashing |

The `.hex` and `.bin` are produced from the same `firmware.elf`, so they are
byte-equivalent. `.hex` is more forgiving over a noisy UART cable because
each record is self-contained (the bootloader can re-sync mid-flash).

---

## Option 1 — `stm32flash` over UART (you already used this for v2)

Wiring: USB-TTL adapter → STM32
- TTL TX → STM32 PA10 (USART1 RX)
- TTL RX → STM32 PA9  (USART1 TX)
- GND     → STM32 GND
- **Do not** connect 3V3 from the TTL — power the STM32 from its own supply.

Boot mode:
1. Power off the STM32.
2. Hold **BOOT0 HIGH** (paperclip to 3V3).
3. Press and release **NRST** (reset).
4. Keep BOOT0 HIGH.
5. Now run the flash command (see below).
6. After the flash finishes, power-cycle the STM32 with BOOT0 LOW to run normally.

Flash command (Linux):

```bash
# Find the port
ls /dev/ttyUSB*

# Try .hex first (more robust)
stm32flash -w firmware.hex -v -g 0x0 /dev/ttyUSB0

# If the port is wrong baud or has a slow USB chip
stm32flash -b 115200 -w firmware.hex -v -g 0x0 /dev/ttyUSB0

# Raw .bin if .hex fails (older bootloaders want this)
stm32flash -w firmware.bin -v -g 0x0 /dev/ttyUSB0
```

What the flags do:
- `-w firmware.hex` — write this file to flash
- `-v` — verify after writing
- `-g 0x0` — jump to address 0 (i.e. start the new firmware) when done
- `-b 115200` — force baud rate

If you get `Wrong boot loader selected`, the BOOT0 paperclip is in the
wrong state, or NRST wasn't actually pulsing (some boards have a reset
button, some don't — just unplug-replug power).

If you get `Write error` after a few percent, the wire is too long, the
USB-TTL ground isn't shared, or the cables are too close to motors that
are powered. Move the STM32 away from the motor drivers, shorten the
wires, and try again.

---

## Option 2 — `dfu-util` over USB-DFU (no extra adapter needed)

The STM32F411 has a built-in USB-DFU bootloader. If you can wire a USB
cable to the STM32's USB pins directly, you can flash without a USB-TTL.

Wiring: USB cable → STM32
- USB D+ → STM32 **PA12** (USB_DP)
- USB D- → STM32 **PA11** (USB_DM)
- **5V from USB** → STM32 5V (or just share ground if STM32 is already powered)
- GND     → GND

Install `dfu-util` on the other laptop:

```bash
# Debian / Ubuntu
sudo apt install dfu-util

# macOS
brew install dfu-util

# Windows
# choco install dfu-util   OR   download from dfu-util.sourceforge.net
```

Boot into DFU:
1. Set **BOOT0 = HIGH**.
2. Press NRST.
3. Plug the USB cable into the laptop.
4. Verify the device is visible:

```bash
dfu-util -l
# Should show: "Found DFU: [0483:df11] ver 0200, devnum=N, cfg=1, intf=0, alt=0, name="@Internal Flash  /0x08000000/512K"
```

5. Flash:

```bash
dfu-util -a 0 -s 0x08000000 -D firmware.bin
```

6. Power-cycle with BOOT0 LOW.

If `dfu-util -l` shows nothing: BOOT0 isn't HIGH, the USB D+/D- wires
are swapped, or the cable is charge-only. Try a different cable.

---

## Option 3 — OpenOCD via ST-Link V2

We had trouble before because the ST-Link wasn't enumerating on USB. To
try again:

1. Plug the ST-Link V2 into a **USB-2.0 (black) port**, not USB-3.0 (blue).
2. Use a **data USB cable**, not a charge-only one.
3. Check:

```bash
lsusb | grep -i st
# Should show: "STMicroelectronics ST-LINK/V2" with VID 0483
```

4. Connect ST-Link to the STM32: SWDIO→PA13, SWCLK→PA14, GND→GND, and
   optionally 3V3→3V3 (powers the STM32 from the ST-Link).
5. Hold NRST on the STM32 to the ST-Link's RST pin (optional, but helps).
6. OpenOCD command (runs from the directory holding the openocd scripts):

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program firmware.elf verify reset exit"
```

This will use the SWD pins (PA13/PA14) and does NOT need BOOT0. The
ST-Link takes over the chip.

---

## Sanity-checking the flash worked

After the flash, with BOOT0 LOW and the STM32 running, the Jetson should
see a `V3\n` come back when it opens the UART. Quick check on the
Jetson:

```bash
stty -F /dev/ttyTHS1 115200 raw -echo
timeout 2 cat /dev/ttyTHS1 | xxd
# Look for: 00000000: 5633 0a                  V3.
```

If you see `V3.`, the new firmware is alive.

---

## If nothing works

1. Try a different USB-TTL adapter (CP2102, CH340, FTDI — all work).
2. Re-check the 4 wires: TX→RX, RX→TX, GND↔GND. **TX must go to RX on
   the other side**, not TX-to-TX.
3. Measure with a multimeter: STM32 PA9 should sit at ~3.3 V idle (UART
   idle high). If it's at 0 V, the bootloader isn't running (BOOT0 not
   HIGH, or chip is unpowered).
4. Power the STM32 from a known-good 5 V supply (USB charger brick
   works), not from the Jetson. The motor drivers can pull enough
   current to glitch the MCU.
