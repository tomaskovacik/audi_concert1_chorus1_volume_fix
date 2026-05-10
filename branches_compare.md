# Branch Comparison Summary

## `devel` vs `master`

Both the `devel` and `master` branches currently point to the **same commit**, meaning **devel and master are in sync** — there are no differences between them right now.

---

## Is `gala` merged into `devel`? ❌ No

The `gala` branch has **not** been merged into `devel`. The comparison shows significant differences in both directions:

- **`gala` has HW changes** not in `devel` — extensive KiCad PCB updates (board v4.0 changes, component replacements like removing Q1/Q2 transistors, adding U4 MAX809, renaming nets like `/#STATUS_FROM_MCU`, etc.)
- **`devel` has SW changes** not in `gala` — the `.ino` firmware has major additions not present in `gala`:
  - EEPROM support for saving GALA/VOL/TA settings
  - GALA speed sensing via timer interrupt (`galaRising`/`galaFalling`)
  - `send_volume()` / `send_loudness()` refactoring
  - Speed-based automatic volume adjustment logic

**In short:** `gala` contains hardware design work for a newer PCB version, while `devel` has the corresponding firmware work — but the two branches have **diverged** and neither has been fully merged into the other.
