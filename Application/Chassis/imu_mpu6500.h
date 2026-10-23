/**
 * @file imu_mpu6500.h
 * @brief MPU-6500 / ICM-20689 六轴IMU驱动 (SPI5)
 * @version 1.0
 * @date 2026-05-30
 *
 * @note
 *  TDK InvenSense MPU-6500 (或 ICM-20689, 引脚兼容)
 *  - 单芯片 6 轴 (加速度计 + 陀螺仪)
 *  - SPI5: SCK = PF7, MISO = PF8, MOSI = PF9
 *  - 片选: CS = PF6, 中断: INT = PB8
 *  - 加速度计量程 ±16g, 陀螺仪量程 ±2000°/s
 */

#ifndef IMU_MPU6500_H
#define IMU_MPU6500_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported macros -----------------------------------------------------------*/

/* ---- 寄存器地址 ---- */

/* 芯片识别 */
#define MPU6500_WHO_AM_I            0x75   /* 芯片ID, 应为 0x70 */
#define MPU6500_WHO_AM_I_VAL        0x70

/* 电源管理 */
#define MPU6500_PWR_MGMT_1          0x6B   /* 电源管理1 */
#define MPU6500_PWR_MGMT_1_DEVICE_RESET  0x80  /* bit7: 复位所有寄存器 */
#define MPU6500_PWR_MGMT_1_SLEEP         0x40  /* bit6: 睡眠模式 */
#define MPU6500_PWR_MGMT_1_CLKSEL_PLL    0x01  /* 自动选择最佳时钟源(PLL) */

#define MPU6500_PWR_MGMT_2          0x6C   /* 电源管理2 (传感器使能) */

/* 信号路径复位 */
#define MPU6500_SIGNAL_PATH_RESET   0x68   /* 信号路径复位 */

/* 采样率分频器 */
#define MPU6500_SMPLRT_DIV          0x19   /* 采样率 = 1kHz / (1 + div) */

/* 配置寄存器 */
#define MPU6500_CONFIG              0x1A   /* 陀螺仪低通滤波 */
#define MPU6500_GYRO_CONFIG         0x1B   /* 陀螺仪量程配置 */
#define MPU6500_ACCEL_CONFIG        0x1C   /* 加速度计量程配置 */
#define MPU6500_ACCEL_CONFIG_2      0x1D   /* 加速度计低通滤波 */

/* 中断配置 */
#define MPU6500_INT_PIN_CFG         0x37   /* 中断引脚配置 */
#define MPU6500_INT_ENABLE          0x38   /* 中断使能 */

/* 加速度计数据寄存器 (从 0x3B 开始, 共 6 字节) */
#define MPU6500_ACCEL_XOUT_H        0x3B
#define MPU6500_ACCEL_XOUT_L        0x3C
#define MPU6500_ACCEL_YOUT_H        0x3D
#define MPU6500_ACCEL_YOUT_L        0x3E
#define MPU6500_ACCEL_ZOUT_H        0x3F
#define MPU6500_ACCEL_ZOUT_L        0x40

/* 温度数据寄存器 (2 字节) */
#define MPU6500_TEMP_OUT_H          0x41
#define MPU6500_TEMP_OUT_L          0x42

/* 陀螺仪数据寄存器 (从 0x43 开始, 共 6 字节) */
#define MPU6500_GYRO_XOUT_H         0x43
#define MPU6500_GYRO_XOUT_L         0x44
#define MPU6500_GYRO_YOUT_H         0x45
#define MPU6500_GYRO_YOUT_L         0x46
#define MPU6500_GYRO_ZOUT_H         0x47
#define MPU6500_GYRO_ZOUT_L         0x48

/* 用户控制 */
#define MPU6500_USER_CTRL           0x6A   /* 用户控制 (I2C 主模式禁用等) */

/* ---- 量程选择 ---- */

/* 陀螺仪量程 (GYRO_CONFIG 的 bit4:3) */
#define MPU6500_GYRO_FS_SEL_250     (0x00 << 3)  /* ±250°/s  */
#define MPU6500_GYRO_FS_SEL_500     (0x01 << 3)  /* ±500°/s  */
#define MPU6500_GYRO_FS_SEL_1000    (0x02 << 3)  /* ±1000°/s */
#define MPU6500_GYRO_FS_SEL_2000    (0x03 << 3)  /* ±2000°/s */

/* 加速度计量程 (ACCEL_CONFIG 的 bit4:3) */
#define MPU6500_ACCEL_FS_SEL_2G     (0x00 << 3)  /* ±2g  */
#define MPU6500_ACCEL_FS_SEL_4G     (0x01 << 3)  /* ±4g  */
#define MPU6500_ACCEL_FS_SEL_8G     (0x02 << 3)  /* ±8g  */
#define MPU6500_ACCEL_FS_SEL_16G    (0x03 << 3)  /* ±16g */

/* ---- DLPF 带宽配置 ---- */

/* CONFIG 寄存器的 DLPF_CFG (bit2:0), 对应 1kHz 内部采样率 */
#define MPU6500_DLPF_CFG_260HZ      0x00   /* 陀螺仪: 256Hz BW, 8kHz Fs */
#define MPU6500_DLPF_CFG_184HZ      0x01   /* 陀螺仪: 188Hz BW, 1kHz Fs */
#define MPU6500_DLPF_CFG_94HZ       0x02   /* 陀螺仪: 98Hz BW, 1kHz Fs */
#define MPU6500_DLPF_CFG_44HZ       0x03   /* 陀螺仪: 42Hz BW, 1kHz Fs */
#define MPU6500_DLPF_CFG_21HZ       0x04   /* 陀螺仪: 20Hz BW, 1kHz Fs */
#define MPU6500_DLPF_CFG_10HZ       0x05   /* 陀螺仪: 10Hz BW, 1kHz Fs */
#define MPU6500_DLPF_CFG_5HZ        0x06   /* 陀螺仪: 5Hz BW, 1kHz Fs */

/* ---- 采样率分频 ---- */
#define MPU6500_SMPLRT_DIV_1KHZ     0x00   /* 1kHz / (1+0) = 1kHz 输出 */

/* ---- 转换常量 ---- */

/* 加速度计分辨率 (m/s²/LSB)
 *   ±16g 量程: 灵敏度 = 2048 LSB/g
 *   1 LSB = (1/2048) g = (9.80665/2048) m/s²
 */
#define MPU6500_ACCEL_16G_SCALE     (9.80665f / 2048.0f)   /* ≈ 0.0047884 m/s²/LSB */

/* 陀螺仪分辨率 (rad/s/LSB)
 *   ±2000°/s 量程: 灵敏度 = 16.4 LSB/(°/s)
 *   1 LSB = (1/16.4) °/s = (1/16.4) * (PI/180) rad/s
 */
#define MPU6500_GYRO_2000_SCALE     (3.14159265358979323846f / (16.4f * 180.0f)) /* ≈ 0.0010642 rad/s/LSB */

/* 温度转换常量
 *   temp_C = raw / 340.0 + 36.53
 */
#define MPU6500_TEMP_SCALE          (1.0f / 340.0f)
#define MPU6500_TEMP_OFFSET         36.53f

/* Exported types ------------------------------------------------------------*/

/**
 * @brief MPU6500 原始数据 (与 BMI088_Data_t 字段兼容)
 */
typedef struct
{
    float acc_x;    /* 加速度 X (m/s^2) */
    float acc_y;    /* 加速度 Y (m/s^2) */
    float acc_z;    /* 加速度 Z (m/s^2) */
    float gyro_x;   /* 角速度 X (rad/s) */
    float gyro_y;   /* 角速度 Y (rad/s) */
    float gyro_z;   /* 角速度 Z (rad/s) */
    float temp;     /* 温度 (°C) */
    float yaw;      /* Yaw 角 (rad), gyro_z 积分, 范围 [-PI, PI] */
} MPU6500_Data_t;

/**
 * @brief MPU6500 IMU 句柄
 */
typedef struct
{
    SPI_HandleTypeDef    *hspi;            /* SPI 外设句柄 (SPI5) */

    /* 内部变量 */
    uint8_t               tx_buf[2];       /* SPI 发送缓冲 */
    uint8_t               rx_buf[16];      /* SPI 接收缓冲 (最大 14 字节) */

    /* 校准 */
    float                 gyro_bias_z;     /* 陀螺仪 Z 轴零偏 (rad/s), 上电静止采样平均 */

    /* 输出数据 */
    MPU6500_Data_t        data;
} MPU6500_t;

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief MPU6500 IMU 初始化
 *
 * 初始化流程:
 * 1. 器件复位
 * 2. 唤醒并选择 PLL 时钟源
 * 3. 检查 WHO_AM_I (0x70)
 * 4. 配置量程: 加速度计 ±16g, 陀螺仪 ±2000°/s
 * 5. 配置低通滤波: 44Hz BW, 1kHz 采样率
 *
 * @param imu     IMU 句柄指针
 * @param hspi    SPI 外设句柄 (SPI5)
 * @return true 初始化成功, false 失败
 */
bool MPU6500_Init(MPU6500_t *imu, SPI_HandleTypeDef *hspi);

/**
 * @brief 读取加速度计数据 (阻塞式 SPI)
 * @param imu  IMU 句柄指针
 * @note  读取后数据存入 imu->data.acc_x/y/z (m/s²)
 */
void MPU6500_Read_Acc(MPU6500_t *imu);

/**
 * @brief 读取陀螺仪数据 (阻塞式 SPI)
 * @param imu  IMU 句柄指针
 * @note  读取后数据存入 imu->data.gyro_x/y/z (rad/s)
 */
void MPU6500_Read_Gyro(MPU6500_t *imu);

/**
 * @brief 读取温度数据
 * @param imu  IMU 句柄指针
 * @note  读取后数据存入 imu->data.temp (°C)
 */
void MPU6500_Read_Temp(MPU6500_t *imu);

/**
 * @brief 同时读取加速度计和陀螺仪数据 (14 字节突发读取)
 *
 * 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 字节:
 *   acc[6] + temp[2] + gyro[6]
 *
 * @param imu  IMU 句柄指针
 */
void MPU6500_Read_All(MPU6500_t *imu);

/**
 * @brief MPU6500 自检
 * @param imu  IMU 句柄指针
 * @return true WHO_AM_I 正确 (0x70), false 失败
 */
bool MPU6500_SelfTest(MPU6500_t *imu);

/**
 * @brief 积分 gyro_z 更新 yaw 角
 *
 * yaw += gyro_z * dt, 自动 wrap 到 [-PI, PI]
 *
 * @param imu  IMU 句柄指针
 * @param dt   时间间隔 (秒), 例如 10ms 循环传 0.01f
 * @note  需在 MPU6500_Read_Gyro 或 MPU6500_Read_All 之后调用
 */
void MPU6500_Update_Yaw(MPU6500_t *imu, float dt);

/**
 * @brief 陀螺仪零偏校准 (上电静止时调用)
 *
 * 连续采样 N 次, 计算 gyro_z 平均值作为零偏,
 * 存入 imu->gyro_bias_z. 后续 Update_Yaw 会自动扣除.
 *
 * @param imu      IMU 句柄指针
 * @param samples  采样次数 (建议 200~500, 对应 2~5 秒)
 */
void MPU6500_CalibrateGyro(MPU6500_t *imu, int samples);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MPU6500_H */
