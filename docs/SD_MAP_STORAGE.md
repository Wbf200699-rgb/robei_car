# Robei Car SD Storage Layout

The A7-50T onboard SD slot is used in SPI mode. The FPGA implementation must
be pure RTL and must not use an SD, SDIO, AXI, or filesystem IP core.

## Pin assignment

| SD signal | FPGA pin | SPI function |
| --- | --- | --- |
| CLK | E13 | SCLK |
| CMD | E14 | MOSI |
| D0 | F14 | MISO |
| D3 | F16 | CS_N |
| CD | G13 | Card detect |
| D1 | F13 | Unused input with pull-up |
| D2 | E17 | Unused input with pull-up |

## Disk layout

Sector size is fixed at 512 bytes. The first 32 MiB is not part of the
computer-visible partition. FPGA data starts at LBA 2048, leaving the MBR and
the first 1 MiB untouched.

| Region | LBA range | Purpose |
| --- | --- | --- |
| MBR/alignment gap | 0-2047 | MBR and reserved alignment space |
| Superblocks | 2048-2049 | Primary and backup `ROBEISD1` descriptors |
| Hardware probe scratch | 3072 | Destructive block-write/readback test only |
| Map slot A | 4096-4113 | Header, 16 payload sectors, commit sector |
| Map slot B | 8192-8209 | Header, 16 payload sectors, commit sector |
| Calibration | 12288, 12304 | Future redundant calibration records |
| Future raw storage | Remaining LBAs below 65536 | Navigation and logs |
| PC partition | 65536-end | exFAT volume named `ROBEI_SD` |

The authoritative machine-readable constants are in
`config/robei_sd_layout.json`.

## Superblock V1

All integers are unsigned little-endian. CRC32 uses the reflected IEEE
polynomial `0xEDB88320` over bytes 0 through 507. The stored CRC occupies bytes
508 through 511.

| Offset | Size | Value |
| --- | --- | --- |
| 0x000 | 8 | ASCII `ROBEISD1` |
| 0x008 | 4 | Format version, 1 |
| 0x00C | 4 | Sector size, 512 |
| 0x010 | 4 | Raw region start LBA |
| 0x014 | 4 | Raw region sector count |
| 0x018 | 4 | PC partition start LBA |
| 0x01C | 4 | Map payload bytes, 8192 |
| 0x020 | 4 | Map payload sectors, 16 |
| 0x024-0x038 | 24 | A/B header, payload, and commit LBAs |
| 0x03C-0x040 | 8 | Calibration slot LBAs |
| 0x044 | 4 | Feature flags; bit 0 means dual map slots |
| 0x048 | 8 | Initialization Unix timestamp |
| 0x050 | 8 | Total card sectors |
| 0x058 | 16 | ASCII volume description |
| 0x1FC | 4 | Superblock CRC32 |

## Save policy

The live 128 x 128 x 4-bit occupancy map remains in FPGA RAM. SD writes are
only allowed while the car is stopped or after an explicit save request. A new
record is written to the older A/B slot, read back, CRC checked, and committed
last. Loading selects the valid committed slot with the newest sequence.

Do not repartition or reformat the card from Windows after initialization. A
normal file copy to the `ROBEI_SD` exFAT volume does not touch the FPGA region.

`LBA 3072` is explicitly reserved for hardware diagnostics. Production map
save/load logic never accesses it. The write probe fills only this sector,
clears the FPGA-side buffer, reads the sector back, and requires both a zero
byte-mismatch count and equal IEEE CRC32 values before reporting success.

## Map slot V1

Each map slot is committed in this order: invalidate commit sector, write and
read back 16 payload sectors, write and read back the header, then write and
read back the valid commit sector. A power loss before the last operation
therefore leaves the previous slot authoritative.

Header bytes 0-7 are ASCII `LCMAPSD1`. Bytes 8-11 contain version 1; bytes
12-15 contain the monotonically increasing sequence. Bytes 16-19 contain map
width and height (128 x 128), bytes 20-23 describe the 50 mm cell size, bytes
24-29 contain payload size and sector count, and bytes 30-47 contain map epoch,
pose, heading, scan/count metadata, and payload CRC32. Bytes 508-511 contain
the CRC32 of header bytes 0-507.

Commit bytes 0-7 are ASCII `LCMCOMT1`. Bytes 8-20 repeat the sequence, header
CRC32, payload CRC32, and target slot. Byte 21 must be `0xA5`; byte 22 is
version 1. Bytes 508-511 contain the commit-sector CRC32. A slot is loadable
only when header, payload, commit marker, repeated metadata, and every CRC
agree.
