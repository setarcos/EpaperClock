# EpaperClock

An ultra-low-power clock based on an **STM32F051C8** and a **Waveshare 2.9-inch e-Paper display**.
It stays in micro-amp standby and wakes once per minute via an RTC alarm to refresh the time; a single 1000 mAh cell is estimated to last **1.5~2 months**.

## Basic Features

- **Clock display**: large 7-segment-style bitmap digits for "HH:MM", with the date (YY-MM-DD) and weekday (Mon~Sun) on the top line and a colon shown as two fixed dots.
- **RTC timekeeping**: on-chip RTC driven by an external 32.768 kHz LSE crystal; keeps running in the backup domain.
- **Minute-by-minute refresh**: RTC Alarm A fires every minute (second = 0) to wake the MCU and refresh the e-Paper once; the rest of the minute (~58 s) is spent sleeping.
- **Partial refresh + daily full refresh**: uses the partial-update LUT for normal refreshes (faster, lower power); at 00:00 every day one refresh switches to the full-update LUT to clear ghosting (as required by the vendor after many partial refreshes). (Older builds performed an MCU soft reset at exactly 00:00:00 for this; that collided with the RTC calendar's day rollover and could corrupt the hour field, so the reset was replaced by a plain full refresh that never touches the RTC at the rollover instant.)
- **Serial time setting**: interactive date/time entry over UART (115200, 8N1) — see [Serial Time Setting](#serial-time-setting).
- **Dual operating modes**: a hardware switch selects "UART/debug mode (SLEEP)" or "battery mode (STOP)"; battery mode wakes only on the RTC alarm for the lowest possible current.

## Hardware

| Component | Model / Spec |
|-----------|--------------|
| MCU | STM32F051C8T6 (ARM Cortex-M0, 48 KB Flash / 8 KB RAM) |
| Display | Waveshare 2.9" e-Paper, 128×296, SPI interface, partial-refresh capable |
| Crystal | 32.768 kHz LSE (RTC timekeeping) |
| Programmer | ST-Link (or any OpenOCD-supported debugger) |

### Pin Connections

| Function | Pin | Notes |
|----------|-----|-------|
| EPD CS | PB0 | chip select (software controlled) |
| EPD DC | PB2 | data/command select |
| EPD BUSY | PB10 | busy input |
| EPD RST | PB11 | reset |
| SPI1 SCK | PA5 | e-Paper clock |
| SPI1 MOSI | PA7 | e-Paper data (1-line half duplex) |
| USART2 TX / RX | PA2 / PA3 | time-setting serial, 115200, 8N1 |
| LED | PC13 | lit briefly while drawing the frame |
| Mode switch out / in | PB13 / PB12 | closed = UART mode; open = battery mode (PB12 has pull-down) |

> The project is an STM32CubeMX project (`epd.ioc`); pin assignments can be regenerated with CubeMX at any time.

## Key Characteristics

### Ultra-Low Power (designed for battery operation)

- **STOP-mode standby**: battery mode enters `STOP + low-power regulator`, ~**2.5 µA** standby current.
- **RTC-alarm wakeup**: wakes for only ~2 s per minute to refresh; sleeps the rest of the time.
- **e-Paper deep sleep**: after every refresh the display is sent to `DEEP_SLEEP_MODE` (standby < 0.01 µA), which also prevents long-term high-voltage stress that could permanently damage the panel.
- **8 MHz HSI only**: the system runs on the HSI 8 MHz clock with no HSE/PLL, halving the CPU current during the wake window.
- **SysTick suspended while asleep**: avoids unnecessary periodic wakeups.
- **LED discipline**: the LED is on only while drawing the frame buffer.
- **Automatic fallback from UART mode**: if no serial traffic arrives for 5 minutes, the device automatically switches from SLEEP back to STOP so you never forget to flip the switch.

> Battery-life estimate is based on Waveshare's official figures (~10–15 mAh/day with a 1000 mAh cell); measure on your own hardware for exact numbers.

### Few Dependencies — Just a Compiler and OpenOCD

- The only **tools** required are the **ARM GCC toolchain** (`arm-none-eabi-*`) and **OpenOCD**. No IDE, no external build system.
- The STM32 HAL and CMSIS sources live in the `Drivers/` directory, which is **excluded from the repository** (see `.gitignore`) and must be fetched once before building — see [Getting the HAL Drivers](#getting-the-hal-drivers).
- Flashing needs just an ST-Link debugger.

### Other

- Cleanly layered drivers: e-Paper controller driver (`epd2in9.c`) → hardware abstraction (`epdif.c`, isolating SPI/GPIO/delay) → paint library (`epdpaint.c`, strings/bitmaps/rectangles).
- The large 0–9 digit bitmaps (`bitmaps.c`) are plain image data — restyle the clock by swapping the arrays.
- Classic CubeMX-generated Makefile with incremental builds, automatic dependency tracking, and `--gc-sections` dead-code stripping.

## Build

### 1. Install dependencies

Only two tools:

```bash
# Debian / Ubuntu
sudo apt install gcc-arm-none-eabi openocd

# Arch Linux
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib openocd

# macOS (Homebrew)
brew tap ArmMbed/homebrew-formulae
brew install arm-none-eabi-gcc openocd
```

### 2. Get the HAL Drivers

The `Drivers/` folder (STM32 HAL + CMSIS) is not part of the repo. Either:

**Option A (recommended) — regenerate with STM32CubeMX:**

1. Install STM32CubeMX and the STM32CubeF0 firmware package.
2. Open `epd.ioc` in CubeMX.
3. Select *Project → Generate Code*. CubeMX creates `Drivers/` containing `STM32F0xx_HAL_Driver/` and `CMSIS/`.

**Option B — download manually:**

Grab the STM32CubeF0 firmware package from [st.com](https://www.st.com/en/embedded-software/stm32cubef0.html) and copy these folders into the project root:

```
Drivers/STM32F0xx_HAL_Driver/         (Inc/ + Src/, including Inc/Legacy)
Drivers/CMSIS/Device/ST/STM32F0xx/Include/
Drivers/CMSIS/Include/
```

### 3. Compile

```bash
make          # produces build/epd.elf, build/epd.hex, build/epd.bin
```

- The toolchain is expected at `/usr/bin` (see `BINPATH` in the Makefile); edit `BINPATH` or extend `PATH` if it is installed elsewhere.
- Artifacts are written to `build/`.
- Other targets:

```bash
make clean    # remove build/ and .dep/
```

## Flashing

### Via make (recommended)

Connect an ST-Link over SWD, then:

```bash
make prog
```

Equivalent to:

```bash
openocd -d0 -f openocd.cfg \
  -c init -c "reset halt" \
  -c "flash write_image erase build/epd.elf" \
  -c "verify_image build/epd.bin 0x08000000" \
  -c "reset run" -c shutdown
```

This erases, flashes, verifies, and resets the target.

### Manually with OpenOCD

```bash
openocd -f openocd.cfg -c "program build/epd.elf verify reset exit"
```

`openocd.cfg` uses the ST-Link interface with the STM32F0x target:

```
source [find interface/stlink.cfg]
source [find target/stm32f0x.cfg]
```

> For another debugger, just replace `interface/stlink.cfg`.

## Serial Time Setting

1. Close the mode switch and power on to enter UART mode.
2. Connect to **PA2 (TX) / PA3 (RX)** with any serial terminal, **115200, 8N1, no flow control**.
3. Press any key; the device prompts for the date and time in turn:

```
Input Date(yy-mm-dd:w):
Input Time(hh:mm:ss):
Input Done!
```

| Input | Format | Example |
|-------|--------|---------|
| Date | `yy-mm-dd:w` — 10 chars, `w` = weekday (1=Mon … 7=Sun) | `25-08-29:5` |
| Time | `hh:mm:ss` — 8 chars, 24-hour | `21:30:00` |

4. After `Input Done!`, the RTC immediately runs on the new time.
5. If you stop using the serial port, the device **automatically falls back to STOP battery mode after 5 minutes of inactivity**; you can also just open the switch yourself.

> Note: the year is two digits (`25` means 2025).

## License

- The e-Paper drivers (`epd2in9.c`, `epdif.c`, `epdpaint.c`, etc.) come from [Waveshare](https://www.waveshare.com/) under the MIT license.
- The STM32 HAL library and CMSIS are copyright STMicroelectronics and used under ST's license terms.
- The remaining code is original to this project.
