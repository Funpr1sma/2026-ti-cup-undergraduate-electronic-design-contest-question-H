#include "MPU6050.h"

#include <stddef.h>

#include "MPU6050_Reg.h"
#include "clock.h"
#include "ti_msp_dl_config.h"

#define MPU6050_I2C_ADDRESS       0x68U
#define MPU6050_I2C_TIMEOUT_MS    5U

static bool MPU6050_I2C_WaitIdle(void)
{
    uint32_t start = tick_ms;

    while ((DL_I2C_getControllerStatus(I2C_MPU_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((uint32_t)(tick_ms - start) >=
            MPU6050_I2C_TIMEOUT_MS) {
            return false;
        }
    }

    return true;
}

static bool MPU6050_I2C_WaitDone(void)
{
    uint32_t start = tick_ms;

    while ((DL_I2C_getControllerStatus(I2C_MPU_INST) &
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS) != 0U) {
        if ((uint32_t)(tick_ms - start) >=
            MPU6050_I2C_TIMEOUT_MS) {
            return false;
        }
    }

    return true;
}

static bool MPU6050_I2C_Write(
    uint8_t reg_address,
    const uint8_t *data,
    uint8_t length)
{
    uint8_t tx_buffer[8];
    uint8_t tx_length;
    uint8_t written;
    uint8_t index;

    if (data == NULL ||
        length == 0U ||
        length > 7U) {
        return false;
    }

    if (!MPU6050_I2C_WaitIdle()) {
        return false;
    }

    tx_buffer[0] = reg_address;

    for (index = 0U; index < length; index++) {
        tx_buffer[index + 1U] = data[index];
    }

    tx_length = (uint8_t)(length + 1U);

    written = DL_I2C_fillControllerTXFIFO(
        I2C_MPU_INST,
        tx_buffer,
        tx_length);

    if (written != tx_length) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        I2C_MPU_INST,
        MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        tx_length);

    return MPU6050_I2C_WaitDone();
}

static bool MPU6050_I2C_Read(
    uint8_t reg_address,
    uint8_t *data,
    uint8_t length)
{
    uint8_t written;
    uint8_t received = 0U;
    uint32_t start;

    if (data == NULL || length == 0U) {
        return false;
    }

    if (!MPU6050_I2C_WaitIdle()) {
        return false;
    }

    written = DL_I2C_fillControllerTXFIFO(
        I2C_MPU_INST,
        &reg_address,
        1U);

    if (written != 1U) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        I2C_MPU_INST,
        MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1U);

    if (!MPU6050_I2C_WaitDone()) {
        return false;
    }

    if (!MPU6050_I2C_WaitIdle()) {
        return false;
    }

    DL_I2C_startControllerTransfer(
        I2C_MPU_INST,
        MPU6050_I2C_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        length);

    start = tick_ms;

    while (received < length) {
        while (!DL_I2C_isControllerRXFIFOEmpty(
                    I2C_MPU_INST) &&
               received < length) {
            data[received] =
                DL_I2C_receiveControllerData(
                    I2C_MPU_INST);
            received++;
        }

        if ((uint32_t)(tick_ms - start) >=
            MPU6050_I2C_TIMEOUT_MS) {
            return false;
        }
    }

    return MPU6050_I2C_WaitDone();
}

bool MPU6050_WriteReg(
    uint8_t reg_address,
    uint8_t data)
{
    return MPU6050_I2C_Write(
        reg_address,
        &data,
        1U);
}

bool MPU6050_ReadReg(
    uint8_t reg_address,
    uint8_t *data)
{
    if (data == NULL) {
        return false;
    }

    return MPU6050_I2C_Read(
        reg_address,
        data,
        1U);
}

bool MPU6050_ReadRegs(
    uint8_t reg_address,
    uint8_t *data_array,
    uint8_t length)
{
    return MPU6050_I2C_Read(
        reg_address,
        data_array,
        length);
}

bool MPU6050_Init(void)
{
    uint8_t id;

    /*
     * 退出睡眠，使用 X 轴陀螺仪作为时钟源。
     */
    if (!MPU6050_WriteReg(
            MPU6050_PWR_MGMT_1,
            0x01U)) {
        return false;
    }

    if (!MPU6050_WriteReg(
            MPU6050_PWR_MGMT_2,
            0x00U)) {
        return false;
    }

    /*
     * DLPF_CFG = 3：
     * 陀螺仪带宽约 42Hz；
     * 加速度计带宽约 44Hz；
     * 内部采样基准为 1kHz。
     */
    if (!MPU6050_WriteReg(
            MPU6050_CONFIG,
            0x03U)) {
        return false;
    }

    /*
     * 1kHz / (4 + 1) = 200Hz。
     */
    if (!MPU6050_WriteReg(
            MPU6050_SMPLRT_DIV,
            0x04U)) {
        return false;
    }

    /*
     * 陀螺仪量程 +/-500 deg/s。
     * 灵敏度 65.5 LSB/(deg/s)。
     */
    if (!MPU6050_WriteReg(
            MPU6050_GYRO_CONFIG,
            0x08U)) {
        return false;
    }

    /*
     * 加速度计量程 +/-4g。
     * 灵敏度 8192 LSB/g。
     */
    if (!MPU6050_WriteReg(
            MPU6050_ACCEL_CONFIG,
            0x08U)) {
        return false;
    }

    if (!MPU6050_ReadReg(
            MPU6050_WHO_AM_I,
            &id)) {
        return false;
    }

    return (id & 0x7EU) == 0x68U;
}

uint8_t MPU6050_GetID(void)
{
    uint8_t id = 0U;

    (void)MPU6050_ReadReg(
        MPU6050_WHO_AM_I,
        &id);

    return id;
}

bool MPU6050_GetData(
    int16_t *acc_x,
    int16_t *acc_y,
    int16_t *acc_z,
    int16_t *gyro_x,
    int16_t *gyro_y,
    int16_t *gyro_z)
{
    uint8_t data[14];

    if (acc_x == NULL ||
        acc_y == NULL ||
        acc_z == NULL ||
        gyro_x == NULL ||
        gyro_y == NULL ||
        gyro_z == NULL) {
        return false;
    }

    if (!MPU6050_I2C_Read(
            MPU6050_ACCEL_XOUT_H,
            data,
            sizeof(data))) {
        return false;
    }

    *acc_x = (int16_t)(
        ((uint16_t)data[0] << 8) |
        data[1]);

    *acc_y = (int16_t)(
        ((uint16_t)data[2] << 8) |
        data[3]);

    *acc_z = (int16_t)(
        ((uint16_t)data[4] << 8) |
        data[5]);

    *gyro_x = (int16_t)(
        ((uint16_t)data[8] << 8) |
        data[9]);

    *gyro_y = (int16_t)(
        ((uint16_t)data[10] << 8) |
        data[11]);

    *gyro_z = (int16_t)(
        ((uint16_t)data[12] << 8) |
        data[13]);

    return true;
}