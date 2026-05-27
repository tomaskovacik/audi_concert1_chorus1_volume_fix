#!/usr/bin/env python3
"""
Audi Concert1 / Chorus1 — serial decoder
Reads SPI/BTN/I2C lines from the STM32 volume-fix firmware and decodes them.

Usage:
  python3 decoder.py /dev/ttyUSB0          # default 115200
  python3 decoder.py /dev/ttyUSB0 9600
  python3 decoder.py --stdin               # pipe from file / socat
"""

import sys
import re

# ── ANSI colours ─────────────────────────────────────────────────────────────
R  = "\033[91m"   # red    – buttons / knob
G  = "\033[92m"   # green  – volume / loudness
Y  = "\033[93m"   # yellow – display text / LEDs
B  = "\033[94m"   # blue   – I2C
C  = "\033[96m"   # cyan   – SPI raw
RST = "\033[0m"

# ── Panel button map (from audi_concert_panel.h) ─────────────────────────────
BUTTONS = {
    0x01: "1",            0x02: "2",            0x03: "3",
    0x04: "4",            0x05: "5",            0x06: "6",
    0x07: "SEEK_UP / NEXT_CD",
    0x08: "TP",           0x09: "RDS",          0x0A: "CPS",
    0x0B: "MODE",         0x0C: "RD",
    0x0D: "PREV_TRACK / FAST_REWIND",
    0x0E: "FADE",         0x0F: "BALANCE",      0x10: "BASS",
    0x11: "AM",           0x12: "DOLBY",
    0x13: "FAST_FWD / NEXT_TRACK",
    0x14: "TREBLE",       0x15: "AS",           0x16: "SCAN",
    0x17: "FM",
    0x18: "SEEK_DOWN / PREV_CD",
    0x19: "REVERSE",
    0x1A: "REMOTE_VOL_UP",   0x1B: "REMOTE_VOL_DOWN",
    0x1C: "REMOTE_UP",       0x1D: "REMOTE_DOWN",
    0x1E: "CODE_IN",          0x1F: "EJECT",
    0x21: "BUTTON_RELEASE",
    0x25: "PANEL_START",
    0x26: "REMOTE_LEFT",      0x27: "REMOTE_RIGHT",
    0x86: "KNOB_UP",          0x88: "KNOB_DOWN",
}

# ── SPI packet decode (0x9A packets from MC68HC05 → ST6280) ──────────────────
BASS_TABLE = {
    0:  -14, 1: -12, 2: -10, 3: -8, 4: -6, 5: -4, 6: -2,
    7:    0, 8:  14, 9:  12, 10: 10, 11: 8, 12: 6, 13: 4, 14: 2, 15: 0,
}

def decode_spi(d):
    sub = d[1]
    lines = []
    raw = ' '.join(f'{b:02X}' for b in d)

    if sub == 0x13:  # LED indicators
        b2, b3, b4 = d[2], d[3], d[4]
        leds = []
        # byte 2: [nan|FM2|FM1|FM|AS|RDS|REG] (bit 3 = FM band present)
        if b2 & 0x01: leds.append("REG")
        if b2 & 0x02: leds.append("RDS")
        if b2 & 0x04: leds.append("AS")
        if b2 & 0x08:
            if b2 & 0x40: leds.append("FM1")
            if b2 & 0x20: leds.append("FM2")
            if b2 & 0x10: leds.append("AM")
        # byte 2 unknown bits
        if b2 & 0x80: leds.append("b2.7?")
        # byte 3: [?|?|CPS|RD|?|?|presets(2b)]
        mem = b3 & 0x03
        if b3 & 0x04: leds.append("b3.2?")
        if b3 & 0x08: leds.append("b3.3?")   # was wrongly "CPS" — actual meaning unknown
        if b3 & 0x10: leds.append("RD")
        if b3 & 0x20: leds.append("CPS")      # confirmed: lights when CPS active
        if b3 & 0x40: leds.append("b3.6?")
        if b3 & 0x80: leds.append("b3.7?")
        # byte 4: [nan|MODE|AS|SCAN|FM|TP|AM|RDS]
        if b4 & 0x01: leds.append("RDS-led")
        if b4 & 0x02: leds.append("AM-led")
        if b4 & 0x04: leds.append("TP-led")
        if b4 & 0x08: leds.append("FM-led")
        if b4 & 0x10: leds.append("SCAN-led")
        if b4 & 0x20: leds.append("AS-led")
        if b4 & 0x40: leds.append("MODE-led")
        if b4 & 0x80: leds.append("b4.7?")
        lines.append(f"LEDs: {' '.join(leds) or 'none'}  MEM={mem}  [{raw}]")

    elif sub == 0x23:
        lines.append("Display CLEAR")

    elif sub == 0x32:  # AM/FM frequency
        if d[3] == 0x10:
            freq = 531 + d[2] * 9
            lines.append(f"Freq: {freq} kHz (AM)")
        else:
            freq = (875 + d[2]) / 10.0
            lines.append(f"Freq: {freq:.1f} MHz (FM)")

    elif sub == 0x48:  # ASCII display text
        text = ''.join(chr(b) if 0x20 <= b < 0x7F else '.' for b in d[2:])
        lines.append(f"Display: \"{text.rstrip()}\"")

    elif sub == 0x58:  # settings menu text
        text = ''.join(chr(b) if 0x20 <= b < 0x7F else '.' for b in d[2:10])
        lines.append(f"Settings: \"{text.rstrip()}\"")

    elif sub == 0x61:  # status/mode display (tape, AS-store, SAFE, DIAG, BOSE…)
        status = {
            0x00: "Tape: Eject",
            0x01: "Tape: Play ▲",
            0x02: "Tape: Play ▼",
            0x03: "Tape: FF →→",
            0x04: "Tape: RW ←←",
            0x0B: "SAFE",
            0x10: "TP-INFO",
            0x13: "AS-STORE",
            0x14: "DIAG",
            0x17: "Unknown-0x17",
            0x1A: "BOSE",
        }
        lines.append(f"Status: {status.get(d[2], f'0x{d[2]:02X}')}")

    elif sub == 0x71:  # BAS/TRE/BAL/FAD stored value
        hi = d[2] >> 4
        lo = d[2] & 0x0F
        names = {0: "BAS+", 1: "BAS-", 2: "TRE+", 3: "TRE-",
                 4: "BAL-R", 5: "BAL-L", 6: "FAD-F", 7: "FAD-R",
                 0xA: "TP-MEM", 0xB: "GALA"}
        label = names.get(hi, f"0x{hi:X}")
        lines.append(f"{label}: {lo}")

    elif sub == 0x80:
        if d[2] == 0x00:
            lines.append("Shutdown")
        else:
            lines.append(f"Power: 0x{d[2]:02X}")

    elif sub == 0x92:
        lines.append(f"Safe code: {d[2]:02X}{d[3]:02X}")

    elif sub == 0xA2:  # CD changer
        lines.append(f"CD {d[2]:X}  Track {d[3]:X}")

    elif sub == 0xE1:
        # Panel lifecycle states — ST6280 OTP firmware, empirically observed.
        # Upper nibble always 0xF; lower nibble appears to be active-low status flags.
        states = {
            0xFF: "Panel idle / sleep",
            0xFE: "Panel HW init",
            0xFC: "Panel display init",
            0xFB: "Panel START (app ready)",
        }
        label = states.get(d[2], f"Panel state 0x{d[2]:02X}")
        lines.append(label)

    else:
        lines.append(f"Unknown sub=0x{sub:02X}")

    return lines


# ── I2C packet decode (TDA7342) ───────────────────────────────────────────────
def spk_atten(c):
    if (c & 0x1F) == 0x1F:
        return "Muted"
    low  = (c & 0x07) * 1.25
    high = ((c >> 3) & 0x03) * 10
    return f"{-(low + high):.2f} dB"

def decode_i2c(d):
    size = d[0]
    increments = (size - 1) if d[1] > 0x0F else 1
    lines = []
    for i in range(increments):
        sub = (d[1] & 0x0F) + i
        c   = d[i + 2]
        if sub == 0:
            inp_sel = c & 0x07
            if inp_sel == 0:
                cd_gain = "-6dB" if (c & 0x40) else "0dB"
                sel = f"CD (IN3, {cd_gain} diff gain)"
            elif inp_sel == 1:
                sel = "TAPE (IN2)"
            elif inp_sel == 2:
                sel = "FM/AM (IN1)"
            elif inp_sel == 3:
                sel = "TP (AM mono)"
            else:
                sel = f"0x{inp_sel:02X}"
            gain_bits = (c & 0x18) >> 3
            gain = [11.25, 7.5, 3.75, 0.0][gain_bits]
            lines.append(f"Input: {sel}  gain={gain:.2f}dB")
        elif sub == 1:
            if c > 0x0F:
                lines.append("Loudness: OFF")
            else:
                lines.append(f"Loudness: -{(c & 0xF) * 1.25:.2f} dB")
        elif sub == 2:
            vol = (20 - ((c >> 5) & 0x07) * 10) + (((c >> 2) & 0x07) * -1.25) + ((c & 0x03) * -0.31)
            lines.append(f"Volume: {vol:.2f} dB  (raw=0x{c:02X})")
        elif sub == 3:
            bass   = BASS_TABLE.get(c >> 4, 0)
            treble = BASS_TABLE.get(c & 0x0F, 0)
            lines.append(f"Bass: {bass:+d} dB  Treble: {treble:+d} dB")
        elif sub == 4:
            lines.append(f"Spk LF: {spk_atten(c)}")
        elif sub == 5:
            lines.append(f"Spk LR: {spk_atten(c)}")
        elif sub == 6:
            lines.append(f"Spk RF: {spk_atten(c)}")
        elif sub == 7:
            lines.append(f"Spk RR: {spk_atten(c)}")
        elif sub == 8:
            mute_type = {1: "Soft/fast", 3: "Soft/slow"}.get(c & 0x03, "")
            direct    = " Direct" if (c >> 3) & 1 else ""
            lines.append(f"Mute: {mute_type}{direct}  raw=0x{c:02X}")
        else:
            lines.append(f"sub={sub} val=0x{c:02X}")
    return lines


# ── Main line dispatcher ──────────────────────────────────────────────────────
def handle_line(raw):
    raw = raw.strip()
    if not raw:
        return

    if raw.startswith("SPI "):
        try:
            d = [int(x, 16) for x in raw[4:].split()]
        except ValueError:
            print(f"{C}SPI  {raw[4:]}{RST}")
            return
        hex_str = ' '.join(f'{b:02X}' for b in d)
        decoded = decode_spi(d) if d else []
        if decoded:
            for line in decoded:
                # 0x13 LEDs already embed raw; others get it appended here
                suffix = f"  [{hex_str}]" if "[" not in line else ""
                print(f"{Y}SPI  {line}{suffix}{RST}")
        else:
            hex_str = ' '.join(f'{b:02X}' for b in d)
            print(f"{C}SPI  [{hex_str}]{RST}")

    elif raw.startswith("BTN "):
        try:
            code = int(raw[4:], 16)
        except ValueError:
            print(f"{R}BTN  {raw[4:]}{RST}")
            return
        name = BUTTONS.get(code)
        if name:
            print(f"{R}BTN  {name}{RST}")
        else:
            print(f"{R}BTN  0x{code:02X}{RST}")

    elif raw.startswith("I2C "):
        try:
            d = [int(x, 16) for x in raw[4:].split()]
        except ValueError:
            print(f"{B}I2C  {raw[4:]}{RST}")
            return
        decoded = decode_i2c(d) if len(d) >= 3 else []
        if decoded:
            for line in decoded:
                print(f"{G}I2C  {line}{RST}")
        else:
            hex_str = ' '.join(f'{b:02X}' for b in d)
            print(f"{B}I2C  [{hex_str}]{RST}")

    elif raw.startswith("Firmware"):
        print(f"  {raw}")
    else:
        print(f"  {raw}")


def main():
    args = sys.argv[1:]

    if not args or args[0] == "--stdin":
        print("Reading from stdin…")
        for line in sys.stdin:
            handle_line(line)
        return

    port = args[0]
    baud = int(args[1]) if len(args) > 1 else 115200

    try:
        import serial
    except ImportError:
        print("pip install pyserial")
        sys.exit(1)

    print(f"Opening {port} @ {baud}…")
    with serial.Serial(port, baud, timeout=1) as ser:
        ser.reset_input_buffer()   # discard any partial line sitting in the buffer
        ser.readline()             # throw away the first (potentially incomplete) line
        print("Connected. Ctrl-C to quit.\n")
        while True:
            try:
                line = ser.readline().decode("ascii", errors="replace")
                handle_line(line)
            except KeyboardInterrupt:
                print("\nBye.")
                break


if __name__ == "__main__":
    main()
