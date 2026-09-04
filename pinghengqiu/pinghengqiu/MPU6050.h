#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

bool MPU6050_WriteReg(uint8_t reg_address, uint8_t data);

bool MPU6050_ReadReg(
    uint8_t reg_address,
    uint8_t *data);

bool MPU6050_ReadRegs(
    uint8_t reg_address,
    uint8_t *data_array,
    uint8_t length);

bool MPU6050_Init(void);

uint8_t MPU6050_GetID(void);

bool MPU6050_GetData(
    int16_t *acc_x,
    int16_t *acc_y,
    int16_t *acc_z,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z);

#endif