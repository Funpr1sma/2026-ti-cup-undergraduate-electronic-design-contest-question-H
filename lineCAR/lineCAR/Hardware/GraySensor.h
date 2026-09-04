#ifndef HARDWARE_GRAY_SENSOR_H_
#define HARDWARE_GRAY_SENSOR_H_

#include <stdbool.h>
#include <stdint.h>

/**
 * 八路灰度传感器物理排列：
 *
 * 车头左侧                                      车头右侧
 *   S1     S2     S3     S4     S5     S6     S7     S8
 *   PB5    PB15   PB16   PB12   PB13   PB23   PB26   PB27
 *
 * 掩码定义：
 *
 * bit0 -> S1，最左
 * bit1 -> S2
 * bit2 -> S3
 * bit3 -> S4
 * bit4 -> S5
 * bit5 -> S6
 * bit6 -> S7
 * bit7 -> S8，最右
 *
 * 新传感器原始电平：
 *
 * 黑线 -> 低电平 -> 0
 * 白色 -> 高电平 -> 1
 *
 * GraySensor_ReadMask()会在GPIO读取入口统一反相，
 * 因此项目内部mask仍保持：
 *
 * 黑线 -> 1
 * 白色 -> 0
 */

/**
 * @brief 灰度传感器识别出的图案类型。
 */
typedef enum
{
    /**
     * 八路均未检测到黑线。
     */
    GRAY_PATTERN_LOST = 0,

    /**
     * 检测到一段普通连续黑线。
     *
     * 通常为1～3个相邻传感器有效。
     */
    GRAY_PATTERN_NORMAL,

    /**
     * 检测到较宽的一段连续黑线。
     *
     * 通常为4～7个相邻传感器有效。
     */
    GRAY_PATTERN_WIDE,

    /**
     * 同时检测到两段或更多不连续黑线。
     *
     * 例如S1和S8同时有效。
     */
    GRAY_PATTERN_SPLIT,

    /**
     * 八路传感器全部检测到黑线。
     */
    GRAY_PATTERN_ALL_BLACK

} GraySensorPattern_t;

/**
 * @brief 一次灰度传感器采样的完整结果。
 */
typedef struct
{
    /**
     * 八路原始状态掩码。
     *
     * bit0对应S1；
     * bit7对应S8。
     */
    uint8_t mask;

    /**
     * 检测到黑线的传感器数量。
     *
     * 范围：0～8。
     */
    uint8_t activeCount;

    /**
     * 不连续黑线区域的数量。
     *
     * 例如：
     * 00011000 -> 1段
     * 10000001 -> 2段
     * 10101010 -> 4段
     */
    uint8_t groupCount;

    /**
     * 是否至少有一路检测到黑线。
     */
    bool lineDetected;

    /**
     * position是否有效。
     *
     * 丢线时为false。
     */
    bool positionValid;

    /**
     * 黑线相对于车体中心的位置。
     *
     * 最左约为-3500；
     * 中心为0；
     * 最右约为+3500。
     */
    int16_t position;

    /**
     * 当前灰度图案类型。
     */
    GraySensorPattern_t pattern;

} GraySensorData_t;

/**
 * @brief 初始化灰度传感器模块。
 *
 * GPIO方向和引脚复用由SYSCFG_DL_init()完成。
 */
void GraySensor_Init(void);

/**
 * @brief 读取八路灰度传感器的统一逻辑掩码。
 *
 * 函数内部已将新传感器的低电平有效信号反相。
 * 返回值始终使用：黑线=1，白色=0。
 *
 * @return bit0对应最左S1，bit7对应最右S8。
 */
uint8_t GraySensor_ReadMask(void);

/**
 * @brief 读取并分析一次灰度传感器数据。
 *
 * @return 本次完整采样结果。
 */
GraySensorData_t GraySensor_ReadData(void);

/**
 * @brief 获取最近一次周期任务保存的数据。
 *
 * @return 最近一次完整采样结果。
 */
GraySensorData_t GraySensor_GetLastData(void);

/**
 * @brief 灰度传感器周期任务。
 *
 * 应在主循环中持续调用。
 * 模块内部每100 ms完成一次读取、分析和串口输出。
 *
 * @param nowMs 当前系统毫秒数。
 */
void GraySensor_Task(uint32_t nowMs);

#endif /* HARDWARE_GRAY_SENSOR_H_ */