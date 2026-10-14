# SD Map Storage Validation

Date: 2026-08-10

## Result

The onboard SD map-storage path has passed RTL simulation, Robei/Yosys ABC9,
Robei native placement and routing, Vivado implementation, and a physical
single-sector write/readback test. The diagnostic wrote only reserved raw
sector LBA 3072. It did not access the exFAT partition, map slots, calibration
slots, or FPGA configuration Flash.

The physical card returned two identical reports:

```text
@SDW,011,1,1,1,00,00,E5,1,0002,00000C00,0000,C3,3C,77C66EA4,77C66EA4
@SDW,011,1,1,1,00,00,E5,1,0002,00000C00,0000,C3,3C,77C66EA4,77C66EA4
```

Decoded fields:

- card pins were CD/D1/D2 = 0/1/1;
- initialization, completion, and comparison result were all 1;
- controller error and final R1 were both `0x00`;
- write response was `0xE5`; its low five bits are the accepted token `0x05`;
- CCS was 1, so LBA block addressing was active;
- the card became ready after two ACMD41 attempts;
- the tested address was `0x00000C00`, decimal LBA 3072;
- all 512 bytes matched, with first/last bytes `0xC3` and `0x3C`;
- expected and readback CRC32 were both `0x77C66EA4`.

Raw capture:

`little_car/test_results/sd_rw_hardware_20260810_202012.txt`

## Persistence design

The live 128 x 128 x 4-bit occupancy grid remains in FPGA RAM. A save writes
the older A/B slot in this order: invalid commit, 16 payload sectors, header,
then valid commit. Every payload sector and both metadata sectors are read back
before commit. Loading validates header CRC, payload CRC, commit CRC, repeated
sequence/CRC fields, and marker `0xA5`, then chooses the newest valid sequence.

Simulation results:

- SD block device: CMD24/CMD17, accepted write response, busy release, and LBA
  3072 readback passed;
- map controller: 24 writes and 90 reads passed, including a simulated power
  cut that preserved slot A while leaving slot B uncommitted;
- full stack: save, card power-cycle, newest-slot scan, 8192-byte payload load,
  and metadata recovery passed;
- occupancy map: automatic load no longer races the map-clear state machine.

## Tool-flow evidence

Robei/Yosys ABC9 completed with zero `check` problems. The vehicle top had
37,301 cells, an estimated 16,651 logic cells, 24 `RAM64M`, two `RAMB18E1`, and
two `RAMB36E1`. The 512-byte SD sector buffer therefore remained distributed
RAM instead of expanding into flip-flops.

Robei Roplace completed in 360 seconds. Routing converged at iteration 9 with
zero overused resources. Packed utilization was 45,314/65,200 `SLICE_LUTX`
(69%), 10,328/65,200 `SLICE_FFX` (15%), and 3,614/8,150 `CARRY4` (44%). This
run used Robei's 12 MHz timing target and proves native placement/routing, not
50 MHz timing closure.

Vivado independently routed all 29,167 nets with zero routing errors and made
the production bitstream with zero errors or critical warnings. Route usage
was 21,577/32,600 LUTs (66.19%), 9,994 registers (15.33%), three BRAM tiles,
and nine inferred DSPs. Worst setup slack at 50 MHz was +0.487 ns. No XCI,
DCP IP product, SD IP, filesystem IP, or AXI IP was used.

Evidence logs:

- `little_car/test_results/vehicle_sd_robei_yosys_20260810.log`
- `little_car/test_results/vehicle_sd_roplace_20260810.log`
- `little_car/build/vivado_main/route_status.rpt`
- `little_car/build/vivado_main/timing_route.rpt`
- `little_car/build/vivado_main/utilization_route.rpt`

## Programmed artifacts

The diagnostic bitstream SHA-256 was
`B445F0140F1DA9036D190B5533703EE7C879B5D7C2F0927F23A45A9BF55180D7`.
After capture, the SD-enabled production vehicle bitstream was restored to FPGA
SRAM successfully. Its SHA-256 is
`A62AC01A8ECC0D923731982CC74141A67F2089BDF15D4278BAD499C30C2070DA`.
Neither bitstream was written to configuration Flash.
