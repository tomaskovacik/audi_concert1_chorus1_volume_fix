# Firmware — Audi Concert1 / Chorus1 volume fix

STM32F103C6-based firmware that fixes the volume problem on Blaupunkt-made
Audi Concert 1 / Chorus 1 head units.  Target hardware: **HWv5**.

---

## Branches

| Branch | Description |
|---|---|
| `master` | Stable HWv5 firmware — passive SPI sniffer, GALA, flash config, serial decoder |
| `dual-spi-panel-bridge` | **WIP** — HWv5 man-in-the-middle bridge: SPI1 slave (MCU side) + SPI2 master (panel side); STM32 intercepts and forwards packets in both directions. Activate with `#define HWV5_BRIDGE`; omit for passive-sniff fallback. |
| `feature/eeprom-gala-isr-STM32CORE-sleep` | Deep sleep on displayRESET LOW — kept for reference, no real benefit with current HW |

---

## Building and flashing

### Dependencies

- Arduino IDE install at `/opt/arduino-nightly/`
- **Official STMicroelectronics STM32 core 2.12+** installed via Arduino Boards Manager:
  - Board URL: `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
  - Package: `STMicroelectronics:stm32` — select board **Generic STM32F1 series → BluePill F103C6**
  - Tools installed automatically: `xpack-arm-none-eabi-gcc 14.2.1-1.1`, `STM32Tools 2.4.0`, `xpack-openocd 0.12.0-6`
- **FlexWire** library (software I2C) — install via Arduino Library Manager or from source

### Build targets

```bash
cd SW/audi_volume_fix_stm32
make          # compile only
make upload   # compile + flash via ST-Link (OpenOCD)
make serial   # open serial monitor (115200 baud)
make clean    # remove build artefacts
```

### Flashing

Flashing uses **OpenOCD** (bundled with the official core) via ST-Link:

```bash
make upload
```

No separate `st-flash` installation needed.

---

## Serial decoder (`SW/decoder.py`)

Reads the firmware's serial output and prints human-readable labels.

```bash
# From USB serial port (default 115200 baud):
python3 decoder.py /dev/ttyUSB0

# Custom baud:
python3 decoder.py /dev/ttyUSB0 9600

# Pipe from file or socat:
python3 decoder.py --stdin < log.txt

# Append raw hex [XX XX ...] to every decoded line:
python3 decoder.py --debug /dev/ttyUSB0
python3 decoder.py --debug --stdin
```

Output prefixes:

| Prefix | Colour | Meaning |
|---|---|---|
| `SPI` | Yellow | Front-panel → MCU SPI packet (decoded) |
| `BTN` | Red | Button / knob event |
| `TDA` | Green | STM32 → TDA7342 I2C command (decoded) |

---

## Configuration (`feature/eeprom-gala-isr-STM32CORE`)

### Stored values

Configuration is saved in the **last 1 KB page of STM32 flash** (`0x8007C00`).
Three values are persisted:

| Field | Range | Default | Meaning |
|---|---|---|---|
| `vol` | 1–5 | 3 | Start volume level at power-on |
| `gala` | 0–5 | 0 | GALA speed-volume aggressiveness (0 = off) |
| `ta` | 1–5 | 3 | TA (traffic announcement) level |

The page is protected by 3 magic bytes and a CRC (`vol+gala+ta`).
After a CRC pass each field is range-checked; out-of-range fields fall
back to their default individually (guards against partial flash corruption).

### Start volume (`vol`)

`vol` is a **fixed level index**, not a relative offset.  It maps to an
absolute TDA7342 register value via a lookup table:

| `vol` | I2C hex | Approx. level |
|---|---|---|
| 1 | `0x56` | quietest |
| 2 | `0x52` | |
| 3 | `0x4E` | default |
| 4 | `0x4A` | |
| 5 | `0x46` | loudest |

Each step is 4 register counts ≈ **2 dB**.

### GALA speed-volume (`gala`)

GALA (*Geschwindigkeitsabhängige Lautstärkeautomatik* — speed-dependent automatic
volume) automatically raises the volume as speed increases to compensate for rising
road and wind noise, keeping perceived audio level consistent.

| Level | Sensitivity | Description |
|---|---|---|
| `0` | Off | Volume stays exactly where you set it — GALA disabled |
| `1` | Lowest | Subtle boost; only noticeable at high highway speeds |
| `2` | Low | Mild adjustment for relaxed highway driving |
| `3` | Medium | Factory sweet spot for a stock Audi cabin; smooth steps with normal acceleration |
| `4` | High | More aggressive; volume rises at lower speed thresholds |
| `5` | Highest | Aggressively boosts volume starting at low speeds |

When `gala > 0` the firmware measures a VSS (vehicle speed signal) pulse width on
**PB5** (TIM3_CH2, tied with PB4/TIM3_CH1). Speed is derived from the pulse half-period:

```
speed_km/h = 1 000 000 / (2 × avg_pulse_width_µs)
```

An 8-sample rolling average suppresses µs-level jitter.

GALA level controls the base speed threshold at which volume starts rising:

| `gala` | Base threshold |
|---|---|
| 1 | 100 km/h |
| 2 | 85 km/h |
| 3 | 70 km/h |
| 4 | 55 km/h |
| 5 | 40 km/h |

Above the base threshold volume steps **up** by 1 and loudness steps **down**
every additional 30 km/h band.  Slowing down reverses the corrections.

Hardware: BC558 PNP high-side driver on the radio PCB drives the GALA line.
STM32 PB5 is connected in parallel with HC05 pin 23 via a 1 kΩ series resistor.
`INPUT_PULLDOWN` is used so the pin reads LOW cleanly when the transistor is OFF.

### Setting config via serial commands

With `USE_SERIAL` enabled, send commands over UART1 (115200 baud):

| Command | Effect |
|---|---|
| `s1` … `s5` | Set start volume level |
| `g0` … `g5` | Set GALA level (0 = off) |
| `w` | Save current config to flash |

### Setting config via radio menu (display auto-save)

The firmware sniffs `0x9A 0x58` display packets sent by the radio when the
user navigates the **settings menu** (accessed with the TP button).
Config is saved to flash automatically as the user scrolls:

| Radio display | Saved value |
|---|---|
| `VOL  1` … `VOL  5` | `vol` = 1–5 |
| `GALA 1` … `GALA 5` | `gala` = 1–5 |
| `GALA OFF` | `gala` = 0 |
| `TA   1` … `TA   5` | `ta` = 1–5 |

### Debug serial output

When `USE_SERIAL` is defined, key events print with these prefixes:

| Prefix | When |
|---|---|
| `CFG_LOAD: vol=X gala=X ta=X` | Config read from flash at startup |
| `CFG_PANEL: vol/gala/ta=X` | Value parsed from radio display packet |
| `CFG_SAVE: OK/FAIL vol=X gala=X ta=X` | Flash write result |
| `GALA_SPEED: prev->new km/h base_thr=X` | Speed change detected |
| `GALA_VOL: UP/DOWN band=X thr=X vol=0xXX` | Volume step applied |
| `GALA_LOUD: UP/DOWN band=X thr=X loud=0xXX` | Loudness step applied |
