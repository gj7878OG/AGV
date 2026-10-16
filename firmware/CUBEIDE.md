# Flashing v3 with STM32CubeIDE

You don't need to do a build with the toolchain by hand — CubeIDE will
build the project and push it to the chip with one click. Here's the
shortest path.

## What you'll copy off the Jetson

Copy the whole `firmware/cubeide_src/` folder to the laptop that has
STM32CubeIDE installed. It contains:

```
cubeide_src/
├── Inc/
│   ├── main.h
│   └── stm32f4xx_hal_conf.h
└── Src/
    ├── main.c                  ← the application (was main_v3.c)
    └── stm32f4xx_hal_msp.c     ← adds HAL_TIM_MspPostInit for PA0/PA1
```

You do NOT need to copy `main_v1.c`, `main_v2.c`, `firmware.bin`,
`firmware.hex`, the `Makefile`, or `system_stm32f4xx.c` — CubeIDE
generates its own copies of those.

## Steps in CubeIDE

### 1. Create a new project (one-time)

1. Open STM32CubeIDE.
2. **File → New → STM32 Project**.
3. In the chip selector, type `STM32F411RE` and pick the **NUCLEO-F411RE**
   (the chip is identical to your bare STM32F411RET6).
4. Click **Next → Next** until you reach **Code Generation** options.
5. Tick **"Generate peripheral initialization as a pair of '.c/.h' files per
   peripheral"** and untick everything else.
6. Click **Finish**. CubeIDE will create a project with default
   initialization.

### 2. Replace the auto-generated files

In the **Project Explorer** panel on the left, you'll see
`<your-project>/Src/main.c` and `<your-project>/Inc/main.h`. Right-click
each file in turn and **Delete** (say "No" if it asks to delete from
disk; we just want to remove it from the project).

Then **drag-and-drop** the files from the `cubeide_src/` folder into the
matching folders in the project:

- `cubeide_src/Src/main.c`  → project `Src/`
- `cubeide_src/Src/stm32f4xx_hal_msp.c` → project `Src/` (overwrite)
- `cubeide_src/Inc/stm32f4xx_hal_conf.h` → project `Inc/` (overwrite)
- `cubeide_src/Inc/main.h` → project `Inc/`

When asked, choose **"Copy files"** and **"Overwrite without warning"**.

If CubeIDE auto-generates new `main.c` / `main.h` on every project sync
(annoying default behaviour), right-click the project → **Properties →
C/C++ Build → Builder Settings** and ensure "Generate default source
files" is OFF, or just keep our files open in the editor.

### 3. Pick the right debug probe

1. Right-click the project → **Run As → Run Configurations…**
2. Double-click **STM32 Cortex-M C/C++ Application**.
3. On the **Debugger** tab:
   - **Debug probe**: pick **ST-LINK (ST-LINK/V2-1)** if your ST-Link is
     finally showing up; otherwise pick **Serial Wire Debugger (OpenOCD)**
     or whatever CubeProgrammer sees.
   - **Interface**: **SWD**
4. Click **Apply**, then **Close** (don't run yet).

### 4. Build

Press **Ctrl+B** (or **Project → Build All**).

The Console should finish with:
```
text    data     bss     dec     hex filename
7812      16    1880    9708    25ec main.elf
Finished building: main.elf
```

Sizes should match the values from the pre-built `firmware.elf`. If
they don't, the project pulled in extra files; double-check the
`Src/` folder.

### 5. Flash

**Run → Run** (or **F11**). CubeIDE will:
1. Connect to the ST-Link.
2. Erase the chip.
3. Program flash.
4. Reset and start the firmware.

Watch the Console — you should see `Download verified successfully` and
then the green bug-icon (debug session) appears. Press the red square
(stop) once the chip is running, otherwise breakpoints will keep
suspended.

### 6. Verify

Power-cycle the STM32 with **BOOT0 LOW** and look for the boot message
on the Jetson's serial console:

```bash
stty -F /dev/ttyTHS1 115200 raw -echo
timeout 2 cat /dev/ttyTHS1 | xxd
# Expect: 00000000: 5633 0a    V3.
```

## If CubeIDE complains "no ST-Link found"

Same diagnostic as before:
1. Plug into a **USB-2.0 (black) port** on the laptop.
2. Use a **data USB cable** (not a charge-only cable).
3. **Help → ST-LINK Upgrade** — sometimes the ST-Link's firmware needs
   a refresh before it enumerates.
4. Check `lsusb | grep -i st` from a terminal.

## If you get "Target is not responding"

The ST-Link can't talk to the chip. Check:
- SWDIO → STM32 **PA13**
- SWCLK → STM32 **PA14**
- GND → STM32 **GND** (mandatory; don't skip)
- Optional: 3V3 from ST-Link to STM32 3V3 to power the chip from USB

## If you'd rather use the UART bootloader anyway

The pre-built `firmware.hex` at `firmware/firmware.hex` is byte-identical
to what CubeIDE will build. `stm32flash -w firmware.hex -v -g 0x0
/dev/ttyUSB0` is one command. See `firmware/FLASHING.md` §"Option 1".
