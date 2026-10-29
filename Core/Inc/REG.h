/* WITMOTION register IDs used by this JY901S UART port. */
#ifndef WIT_REG_H
#define WIT_REG_H

#define REGSIZE        0x90U
#define SAVE           0x00U
#define RSW            0x02U
#define RRATE          0x03U
#define BAUD           0x04U
#define AXIS6          0x24U
#define READADDR       0x27U
#define GX             0x37U
#define GY             0x38U
#define GZ             0x39U
#define Roll           0x3dU
#define Pitch          0x3eU
#define Yaw            0x3fU
#define KEY            0x69U
#define KEY_UNLOCK     0xb588U

#define WIT_GYRO       0x52U
#define WIT_ANGLE      0x53U
#define WIT_REGVALUE   0x5fU

#endif
