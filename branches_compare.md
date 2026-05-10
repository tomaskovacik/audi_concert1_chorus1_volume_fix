# Branch Comparison Summary

## `devel` vs `master` — Branch Comparison

The `devel` branch is ahead of `master` with significant changes to `SW/audi_volume_fix_stm32/audi_volume_fix_stm32.ino` and one new binary file.

## 📁 Files Changed

- `SW/audi_volume_fix_stm32/audi_volume_fix_stm32.ino` — Major changes (details below)
- `gala_on_V1.png` (new file) — A new image added (likely documentation for GALA on V1 board)

## 🔧 Key Code Changes in `devel`

### 1. EEPROM Support Added

- Includes `<EEPROM.h>` and defines EEPROM layout (`EEPROM_PAGE0_BASE`, `EEPROM_PAGE1_BASE`, `EEPROM_PAGE_SIZE`)
- New EEPROM keys: `EEPROM_GALA`, `EEPROM_VOL`, `EEPROM_TA`, `EEPROM_CR1`–`CR4`
- New functions: `checkEEPROM()`, `getGalaEeprom()`, `getVolEeprom()`, `getTaEeprom()`, `saveGalaEeprom()`, `saveVolEeprom()`, `saveTaEeprom()`
- Settings (GALA, VOL, TA) are now persisted across power cycles

### 2. GALA (Speed-Dependent Volume) Feature

- New GALA pin defined (`PA14` for V1 board, `PA0` for V2)
- Interrupt-driven speed sensing using Timer2 — `galaRising()` / `galaFalling()` ISRs
- Speed is calculated from pulse timing (`captime`)
- Volume and loudness are automatically adjusted as speed increases/decreases in 30 km/h steps (up to 300+ km/h range)
- `getGalaStartSpeed()` computes the activation threshold from EEPROM

### 3. `set_volume()` renamed to `send_volume()`

- `set_loudness()` now only calculates the loudness value
- New `send_loudness()` function handles sending it over I²C
- Calls to `set_volume()` replaced throughout with `send_volume()` + `send_loudness()` pairs
- Volume packet sending logic refactored (removed duplicate `set_loudness()` calls inside the loop)

### 4. Start Volume from EEPROM

- `start_volume` now initialized from `setStartVolumeFromEeprom()` (maps 1–5 → hex values)
- Was hardcoded `0x82`; now defaults to `0x4E`
- Also re-applied on display reset event

### 5. I²C Dump Mode Improvements

- `dumpI2cDataAndDoNotFix` renamed to `dumpI2cData` — it's now independent from the fix logic (both can run simultaneously)
- `dump_i2c_data()` output format improved: `I2c data: [ xx xx ]`

### 6. Display Data Parsing for Settings

- Detects VOL/TA/GALA settings displayed on the panel and saves them to EEPROM automatically
  - e.g., `"VOL  X  "` → saves volume level, `"GALA X  "` → saves GALA setting

### 7. Code Style / Minor Fixes

- Fixed indentation of several `case` labels in `decode_button_push()`
- Miscellaneous whitespace and comment cleanup

## Summary Table

| Feature | `master` | `devel` |
|---|---|---|
| EEPROM persistence | ❌ | ✅ |
| GALA (speed-adaptive volume) | ❌ | ✅ |
| Start volume | Hardcoded `0x82` | From EEPROM (`0x4E` default) |
| Loudness sending | Inside `set_loudness()` | Separate `send_loudness()` |
| Dump mode decoupled from fix | ❌ | ✅ |
| Settings saved from display | ❌ | ✅ |
