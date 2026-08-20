# `solve/` — ROBOTHACK AI CTF Badge solver files

This folder holds the solver scripts and firmware image for the badge write-up.
For the full walkthrough see
[`../ROBOTHACK_AI_CTF_Badge_Write-up.md`](../ROBOTHACK_AI_CTF_Badge_Write-up.md).

## How the files relate

```
m3mdump.py  ──dump──▶  flash_dump.bin  ──reverse──▶  constants / custom cipher
                                                         │
solve_exploit.py  ──OOB leak key + break RSA──▶  solve <n> <b>  ──▶  Flag
```

| File | Description |
|------|-------------|
| `m3mdump.py` | Dumps MCU memory to a `.bin` via the hidden `m3mdump` serial command unlocked by Flag 01 (default: `0x08000000` + 128 KiB flash). |
| `flash_dump.bin` | The 128 KiB STM32 flash image produced by `m3mdump.py`; reverse it to recover the RSA constants and the custom cipher `rhcdecrypt`. |
| `solve_exploit.py` | Full boss solver: `image peek` negative-offset OOB read leaks the key at `0x1FFF6000` → factor the 128-bit `n` to break RSA → send `solve <n> <b>` to capture the flag. |

## Requirements

```bash
pip install pyserial sympy
```

The badge must be in **AI Interactive** mode (top menu item, press B3); the
firmware only services serial commands there.

## Usage

```bash
# 1) Dump the firmware (optional — flash_dump.bin is already included)
python m3mdump.py                       # → flash_dump.bin

# 2) One-shot boss solver
python solve_exploit.py                 # auto-detect serial port
python solve_exploit.py --port COM16    # specify the port
python solve_exploit.py --uibuf 0x20000A24   # &uiBuffer for the flashed build (see .map)
```
