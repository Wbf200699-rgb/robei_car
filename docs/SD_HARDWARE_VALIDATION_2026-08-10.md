# A7-50T Onboard SD Read-Only Validation

Date: 2026-08-10

## Scope

This test validates the onboard SD slot, SPI pin assignment, SDXC
initialization, raw-sector reads, and FPGA CRC32 calculation. It does not issue
`CMD24`, erase commands, filesystem writes, or configuration-Flash writes.

## Hardware

- FPGA: XC7A50T-FGG484
- Card: nominal 128 GB SDXC
- Debug output: onboard FT4232H channel C, COM20, 115200 8N1
- SPI pins: E13 CLK, E14 MOSI/CMD, F14 MISO/D0, F16 CS_N/D3
- Auxiliary inputs: G13 card detect, F13 D1, E17 D2

The card was initialized on the host as `ROBEI_SD_RAW_V1`. FPGA raw storage is
LBA 2048 through 65535, while the exFAT partition starts at LBA 65536.

## Verification stages

1. Icarus simulation passed SD V2 initialization, SDHC addressing, primary and
   backup superblock reads, and CRC32 comparison.
2. Both Verilog modules were converted to Robei `.model` files and converted
   back to Verilog. The round-trip sources compiled successfully.
3. The generated Robei models passed Verilog round-trip compilation and Robei's
   native Yosys `synth_xilinx -abc9` flow with zero `check` errors.
4. Robei's native `Roplace.exe` completed placement and routing at 50 MHz with
   zero overused routes; its post-route estimate was 84.45 MHz.
5. Vivado 2025.2 synthesis, placement, routing, DRC, and bitstream generation
   completed with zero errors and zero critical warnings.
6. The diagnostic bitstream was programmed to FPGA SRAM only.
7. COM20 returned two identical report sets from the physical card.
8. The normal vehicle bitstream was restored after capture.

## Hardware report

```text
@SD0,011,11,00,01,01,00,0063,00,1
@SD1,000001AA,C0FF8000,00,FE,524F424549534431,00000001,0354EF70,0354EF70
@SD2,00,FE,524F424549534431,00000001,0354EF70,0354EF70
```

Decoded result:

- Card detect is active low: CD=0 with a card inserted; D1=1 and D2=1.
- Probe done=1, probe OK=1, error=0.
- CMD0 R1=0x01.
- CMD8 R1=0x01 and echo=0x000001AA.
- ACMD41 became ready on attempt 0x0063 (99), R1=0x00.
- CMD58 R1=0x00, OCR=0xC0FF8000, CCS=1; block addressing is active.
- CMD17 returned R1=0x00 and token=0xFE for both LBAs.
- Both sectors contain ASCII `ROBEISD1` and format version 1.
- Stored and FPGA-calculated CRC32 are both 0x0354EF70 for both copies.

Raw capture:

`little_car/test_results/sd_readonly_hardware_20260810_181130.txt`

Robei native-flow logs:

- `little_car/test_results/sd_readonly_robei_yosys_20260810.log`
- `little_car/test_results/sd_readonly_roplace_20260810.log`

## Resource result

- Slice LUTs: 978 (3.00%)
- Slice registers: 892 (1.37%)
- BRAM: 0
- DSP: 0
- Post-route worst setup slack: +10.773 ns

Robei/Roplace used 1,776 `SLICE_LUTX` sites (2%), 907 `SLICE_FFX`
sites (1%), no BRAM, and no DSP. These counts use Roplace's packed-device
accounting and therefore are not directly comparable to Vivado's RTL resource
report.

This document records the earlier read-only stage. The subsequent destructive
scratch-sector test, dual-slot map controller validation, Robei integration,
and restored production firmware are recorded in
`SD_MAP_STORAGE_VALIDATION_2026-08-10.md`.
