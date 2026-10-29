# WITMOTION STM32 NORMAL SDK provenance

Source: https://github.com/WITMOTION/WitStandardProtocol_JY901

Pinned revision: `e8f4e3afc02263e2ef15052dc758f45efaebe12f`.

Upstream directory: `STM32F1/STM32Core_SDK_Normal/Source/wit_c_sdk/`.
The associated `Source/APP/Main.c` and `Source/Drive/UART2.c` were inspected.
The user-provided download service `https://attach.wit-motion.cn/s/7ZHV`
could not be retrieved with certificate verification because its certificate
was expired on 2026-10-25. This official repository is also linked from the
manufacturer's SDK page: https://wit-motion.yuque.com/wumwnr/ltst03/es0tfkrptxfv9dcq .

Imported files and upstream SHA-256:

- `wit_c_sdk.c`: `fc40a5c86e712d059e575d4159b4e69c979f952c54c32b1dfc9588b5617dd547`
- `wit_c_sdk.h`: `4d252a07f5b945b7e1b89d2a6b691d8eec0975c1c75692022ebcd55fa72a0f0d`
- `REG.h`: `9b1c061ccf0bfb43afee87e73c82ab13b67519cd14a09aad8d9cb2601bd370fe`.

Local port and cleanup:

- The project retains only the NORMAL UART path used by JY901S. The supported
  frame types are gyro (0x52), Euler angles (0x53), and four-register readback
  (0x5f). Upstream register addresses, little-endian values, the 11-byte frame,
  additive checksum and five-byte register commands are retained.
- Parsing runs in imuTask and dispatches complete frames immediately. The F1
  example's shared single-frame staging variables and sample UART/main loop
  are not used. Overlapping buffer shifts use memmove.
- Removed unused IIC, CAN and Modbus paths, CRC16 tables, other output types,
  calibration/configuration demo wrappers, delay callback and deinit helpers.
- wit_c_sdk.h and REG.h expose only this port's used APIs and register groups.
  WitReadReg requires count=4 because NORMAL readback always contains four
  registers. WitInit keeps the upstream protocol/address signature for this
  NORMAL-only port; the address is unused on the wire.
- The parser buffer is sized to one 11-byte NORMAL frame. sReg keeps the
  upstream register range for configuration and callback access.

jy901s.c/h provide F407 HAL/FreeRTOS transport, startup configuration readback,
stationary validation, receive timestamps, direct yaw feedback and faults.
Temporary startup diagnostics were removed after the user resolved the module
calibration issue. Original SDK downloads and the pre-cleanup code remain in
backups/jy901s-migration-2026-10-25/ and backups/jy901s-cleanup-2026-10-25/.
