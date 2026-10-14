# Repository Scope Rules

## Current Competition Configuration

- The five VL53L1X laser-ranging channels are not used in the current vehicle configuration.
- Their RTL, Robei models, protocol fields, tests, and historical validation files are retained only for compatibility and traceability.
- Do not describe VL53L1X hardware, five-channel laser ranging, or `@V` telemetry as an enabled, connected, tested, scored, or delivered feature in reports, presentations, handoffs, or generated project documentation.
- Do not remove or modify the retained VL53L1X implementation unless the user explicitly requests a code change.
- Current active obstacle sensing is based on RPLIDAR C1 and ultrasonic sensing. Follow the latest project evidence for every other sensor or feature claim.
