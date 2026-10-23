 /**
 * @file imu_mpu6500.c
 * @brief MPU-6500 / ICM-20689 六轴IMU驱动实现 (SPI5)
 * @version 1.0
 * @date 2026-05-30
 *
 * @note
 *  TDK InvenSense MPU-6500 (或 ICM-20689, 引脚兼容)
 *  SPI5: SCK = PF7, MISO = PF8, MOSI = PF9
 *  片选: CS = PF6, 中断: INT = PB8 (轮询模式, 不使用中断)
 *  SPI 模式: Mode 3 (CPOL=1, CPHA=1)
 */

/* Includes ------------------------------------------------------------------*/

#include "imu_mpu6500.h"
#include "gpio.h"

/* Private defines -----------------------------------------------------------*/

/* SPI 读写标志位: 读=0x80, 写=0x00 */
#define MPU6500_SPI_READ   0x80
#define MPU6500_SPI_WRITE  0x00

/* 芯片选择引脚 PF6, 推挽输出, 上拉, 空闲高 */
#define MPU6500_CS_PORT    GPIOF
#define MPU6500_CS_PIN     GPIO_PIN_6

/* Private macros ------------------------------------------------------------*/

/**
 * @brief 拉低片选 (SPI 传输开始)
 */
#define MPU6500_CS_LOW()   HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_RESET)

/**
 * @brief 拉高片选 (SPI 传输结束)
 */
#define MPU6500_CS_HIGH()  HAL_GPIO_WritePin(MPU6500_CS_PORT, MPU6500_CS_PIN, GPIO_PIN_SET)

/* Private function declarations ---------------------------------------------*/

/**
 * @brief SPI 读写一个字节
 * @param hspi  SPI 句柄
 * @param tx    发送的字节
 * @return 接收到的字节
 */
static uint8_t MPU6500_SPI_ReadWriteByte(SPI_HandleTypeDef *hspi, uint8_t tx)
{
    uint8_t rx = 0xFF;
    if (HAL_SPI_TransmitReceive(hspi, &tx, &rx, 1, 100) != HAL_OK)
    {
        return 0xFF;  /* SPI 超时, 返回无效值 */
    }
    return rx;
}

/**
 * @brief 写 MPU6500 寄存器
 *
 * SPI 写协议: CS 拉低, 发送 address (bit7=0 表示写), 发送 data, CS 拉高
 *
 * @param imu    IMU 句柄
 * @param reg    寄存器地址
 * @param value  要写入的值
 */
static void MPU6500_WriteReg(MPU6500_t *imu, uint8_t reg, uint8_t value)
{
    MPU6500_CS_LOW();
    MPU6500_SPI_ReadWriteByte(imu->hspi, reg & 0x7F); /* bit7=0 表示写 */
    MPU6500_SPI_ReadWriteByte(imu->hspi, value);
    MPU6500_CS_HIGH();
}

/**
 * @brief 读 MPU6500 寄存器 (单字节)
 *
 * SPI 读协议: CS 拉低, 发送 address (bit7=1 表示读), 发送哑字节 0xFF, 接收数据, CS 拉高
 *
 * @param imu  IMU 句柄
 * @param reg  寄存器地址
 * @return 寄存器值
 */
static uint8_t MPU6500_ReadReg(MPU6500_t *imu, uint8_t reg)
{
    uint8_t value;
    MPU6500_CS_LOW();
    MPU6500_SPI_ReadWriteByte(imu->hspi, reg | MPU6500_SPI_READ); /* bit7=1 表示读 */
    value = MPU6500_SPI_ReadWriteByte(imu->hspi, 0xFF);           /* 发送哑字节, 接收数据 */
    MPU6500_CS_HIGH();
    return value;
}

/**
 * @brief 读 MPU6500 多字节数据
 *
 * SPI 多字节读协议: CS 拉低, 发送起始地址 (bit7=1), 连续发送 len 个哑字节接收数据, CS 拉高
 * MPU-6500 支持地址自动递增读取
 *
 * @param imu   IMU 句柄
 * @param reg   起始寄存器地址
 * @param data  数据缓冲区
 * @param len   读取长度
 */
static void MPU6500_ReadMulti(MPU6500_t *imu, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;
    MPU6500_CS_LOW();
    MPU6500_SPI_ReadWriteByte(imu->hspi, reg | MPU6500_SPI_READ);
    for (i = 0; i < len; i++)
    {
        data[i] = MPU6500_SPI_ReadWriteByte(imu->hspi, 0xFF);
    }
    MPU6500_CS_HIGH();
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief MPU6500 初始化
 *
 * 初始化流程:
 * 1. 器件复位 (PWR_MGMT_1 = 0x80)
 * 2. 等待复位完成 (100ms)
 * 3. 唤醒 + 选择 PLL 时钟源 (PWR_MGMT_1 = 0x01)
 * 4. 等待时钟稳定 (100ms)
 * 5. 检查 WHO_AM_I (0x75 == 0x70)
 * 6. 配置陀螺仪量程 ±2000°/s
 * 7. 配置加速度计量程 ±16g
 * 8. 配置 DLPF: 44Hz 带宽, 1kHz 采样率
 * 9. 清零输出数据
 *
 * @param imu     IMU 句柄指针
 * @param hspi    SPI 外设句柄 (SPI5)
 * @return true 初始化成功, false 失败
 */
bool MPU6500_Init(MPU6500_t *imu, SPI_HandleTypeDef *hspi)
{
    uint8_t who_am_i;
    uint8_t retry;

    /* 参数检查 */
    if (imu == NULL || hspi == NULL)
    {
        return false;
    }

    /* 绑定 SPI 句柄 */
    imu->hspi = hspi;

    /* ---- 拉高片选 (空闲状态) ---- */
    MPU6500_CS_HIGH();

    HAL_Delay(10);

    /* ---- 1. 器件复位 ---- */
    MPU6500_WriteReg(imu, MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_DEVICE_RESET);
    HAL_Delay(200);  /* 等待复位完成, 所有寄存器恢复默认值 */

    /* ---- 2. 唤醒 + 选择 PLL 时钟源 ---- */
    /* CLKSEL = 001: 自动选择最佳时钟源 (PLL with X axis gyro reference) */
    /* SLEEP  = 0:   退出睡眠模式 */
    MPU6500_WriteReg(imu, MPU6500_PWR_MGMT_1, MPU6500_PWR_MGMT_1_CLKSEL_PLL);
    HAL_Delay(200);  /* 等待 PLL 锁定和陀螺仪振荡器稳定 */

    /* ---- 3. 检查芯片 ID (重试 5 次, 每次间隔 10ms) ---- */
    for (retry = 0; retry < 5; retry++)
    {
        who_am_i = MPU6500_ReadReg(imu, MPU6500_WHO_AM_I);
        if (who_am_i == MPU6500_WHO_AM_I_VAL)  /* 0x70 */
        {
            break;
        }
        HAL_Delay(10);
    }

    if (who_am_i != MPU6500_WHO_AM_I_VAL)
    {
        /* SPI 通信失败或芯片型号不匹配, 返回 false */
        return false;
    }

    /* ---- 4. 配置陀螺仪 ---- */
    /* 量程: ±2000°/s, FS_SEL = 3 */
    MPU6500_WriteReg(imu, MPU6500_GYRO_CONFIG, MPU6500_GYRO_FS_SEL_2000);

    /* ---- 5. 配置加速度计 ---- */
    /* 量程: ±16g, AFS_SEL = 3 */
    MPU6500_WriteReg(imu, MPU6500_ACCEL_CONFIG, MPU6500_ACCEL_FS_SEL_16G);

    /* ---- 6. 配置 DLPF (低通滤波器) ---- */
    /* DLPF_CFG = 3: 陀螺仪 42Hz BW, 加速度计 44Hz BW, 均为 1kHz 采样率 */
    MPU6500_WriteReg(imu, MPU6500_CONFIG, MPU6500_DLPF_CFG_44HZ);

    /* ---- 7. 配置采样率 ---- */
    /* 内部采样率 1kHz / (1 + 0) = 1kHz */
    MPU6500_WriteReg(imu, MPU6500_SMPLRT_DIV, MPU6500_SMPLRT_DIV_1KHZ);

    /* ---- 8. 禁用 I2C 主模式 (减少功耗和干扰) ---- */
    MPU6500_WriteReg(imu, MPU6500_USER_CTRL, 0x00);

    /* ---- 9. 使能所有传感器 (PWR_MGMT_2 = 0x00, 加速度计和陀螺仪都不关闭) ---- */
    MPU6500_WriteReg(imu, MPU6500_PWR_MGMT_2, 0x00);

    HAL_Delay(10);

    /* ---- 10. 初始化输出数据 ---- */
    imu->data.acc_x  = 0.0f;
    imu->data.acc_y  = 0.0f;
    imu->data.acc_z  = 0.0f;
    imu->data.gyro_x = 0.0f;
    imu->data.gyro_y = 0.0f;
    imu->data.gyro_z = 0.0f;
    imu->data.temp   = 0.0f;
    imu->data.yaw    = 0.0f;
    imu->gyro_bias_z = 0.0f;

    return true;
}

/**
 * @brief MPU6500 自检
 * @param imu  IMU 句柄指针
 * @return true WHO_AM_I 正确, false 失败
 */
bool MPU6500_SelfTest(MPU6500_t *imu)
{
    uint8_t who_am_i;

    if (imu == NULL)
    {
        return false;
    }

    who_am_i = MPU6500_ReadReg(imu, MPU6500_WHO_AM_I);

    if (who_am_i == MPU6500_WHO_AM_I_VAL)
    {
        return true;
    }

    return false;
}

/**
 * @brief 读取加速度计数据
 *
 * MPU-6500 加速度计数据格式: 16-bit 有符号整数, 大端 (MSB 在前)
 * 从 ACCEL_XOUT_H (0x3B) 连续读取 6 字节
 *
 * @param imu  IMU 句柄指针
 */
void MPU6500_Read_Acc(MPU6500_t *imu)
{
    uint8_t  raw[6];
    int16_t  raw_x, raw_y, raw_z;

    if (imu == NULL)
    {
        return;
    }

    /* 连续读取 6 字节: X_H, X_L, Y_H, Y_L, Z_H, Z_L (大端) */
    MPU6500_ReadMulti(imu, MPU6500_ACCEL_XOUT_H, raw, 6);

    /* 拼接为有符号 16-bit 整数 (MSB 在前, 大端) */
    raw_x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    raw_y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    raw_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    /* 转换为 m/s^2 */
    imu->data.acc_x = (float)raw_x * MPU6500_ACCEL_16G_SCALE;
    imu->data.acc_y = (float)raw_y * MPU6500_ACCEL_16G_SCALE;
    imu->data.acc_z = (float)raw_z * MPU6500_ACCEL_16G_SCALE;
}

/**
 * @brief 读取陀螺仪数据
 *
 * MPU-6500 陀螺仪数据格式: 16-bit 有符号整数, 大端 (MSB 在前)
 * 从 GYRO_XOUT_H (0x43) 连续读取 6 字节
 *
 * @param imu  IMU 句柄指针
 */
void MPU6500_Read_Gyro(MPU6500_t *imu)
{
    uint8_t  raw[6];
    int16_t  raw_x, raw_y, raw_z;

    if (imu == NULL)
    {
        return;
    }

    /* 连续读取 6 字节: X_H, X_L, Y_H, Y_L, Z_H, Z_L (大端) */
    MPU6500_ReadMulti(imu, MPU6500_GYRO_XOUT_H, raw, 6);

    /* 拼接为有符号 16-bit 整数 (MSB 在前, 大端) */
    raw_x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    raw_y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    raw_z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    /* 转换为 rad/s */
    imu->data.gyro_x = (float)raw_x * MPU6500_GYRO_2000_SCALE;
    imu->data.gyro_y = (float)raw_y * MPU6500_GYRO_2000_SCALE;
    imu->data.gyro_z = (float)raw_z * MPU6500_GYRO_2000_SCALE;
}

/**
 * @brief 读取温度数据
 *
 * MPU-6500 温度数据格式: 16-bit 有符号整数, 大端
 * 转换公式: temp_C = raw / 340.0 + 36.53
 *
 * @param imu  IMU 句柄指针
 */
void MPU6500_Read_Temp(MPU6500_t *imu)
{
    uint8_t  raw[2];
    int16_t  raw_temp;

    if (imu == NULL)
    {
        return;
    }

    /* 读取 2 字节: TEMP_OUT_H, TEMP_OUT_L */
    MPU6500_ReadMulti(imu, MPU6500_TEMP_OUT_H, raw, 2);

    /* 拼接为有符号 16-bit 整数 (MSB 在前, 大端) */
    raw_temp = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);

    /* 转换为摄氏度 */
    imu->data.temp = (float)raw_temp * MPU6500_TEMP_SCALE + MPU6500_TEMP_OFFSET;
}

/**
 * @brief 同时读取加速度计和陀螺仪数据 (14 字节突发读取)
 *
 * 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 字节:
 *   ACC_X[2] + ACC_Y[2] + ACC_Z[2] + TEMP[2] + GYRO_X[2] + GYRO_Y[2] + GYRO_Z[2]
 *
 * 这比分别调用 Read_Acc + Read_Gyro 更高效,
 * 只需要一次 SPI 事务即可获取所有传感器数据。
 *
 * @param imu  IMU 句柄指针
 */
void MPU6500_Read_All(MPU6500_t *imu)
{
    uint8_t  raw[14];
    int16_t  raw_val;

    if (imu == NULL)
    {
        return;
    }

    /* 从 ACCEL_XOUT_H (0x3B) 开始连续读取 14 字节 */
    MPU6500_ReadMulti(imu, MPU6500_ACCEL_XOUT_H, raw, 14);

    /* ---- 加速度计 (偏移 0~5) ---- */
    /* 大端拼接: MSB << 8 | LSB */

    raw_val = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    imu->data.acc_x = (float)raw_val * MPU6500_ACCEL_16G_SCALE;

    raw_val = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    imu->data.acc_y = (float)raw_val * MPU6500_ACCEL_16G_SCALE;

    raw_val = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    imu->data.acc_z = (float)raw_val * MPU6500_ACCEL_16G_SCALE;

    /* ---- 温度 (偏移 6~7) ---- */

    raw_val = (int16_t)(((uint16_t)raw[6] << 8) | raw[7]);
    imu->data.temp = (float)raw_val * MPU6500_TEMP_SCALE + MPU6500_TEMP_OFFSET;

    /* ---- 陀螺仪 (偏移 8~13) ---- */

    raw_val = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
    imu->data.gyro_x = (float)raw_val * MPU6500_GYRO_2000_SCALE;

    raw_val = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
    imu->data.gyro_y = (float)raw_val * MPU6500_GYRO_2000_SCALE;

    raw_val = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);
    imu->data.gyro_z = (float)raw_val * MPU6500_GYRO_2000_SCALE;
}

/**
 * @brief 积分 gyro_z 更新 yaw 角
 *
 * yaw += gyro_z * dt, 自动 wrap 到 [-PI, PI]
 * 顺时针旋转 (俯视) 产生负向 dyaw (右手定则: gyro_z 负 = CW)
 *
 * @param imu  IMU 句柄指针
 * @param dt   时间间隔 (秒)
 */
void MPU6500_Update_Yaw(MPU6500_t *imu, float dt)
{
    if (imu == NULL)
    {
        return;
    }

    /* 欧拉积分: yaw += (gyro_z - bias) * dt, 扣除零偏 */
    imu->data.yaw += (imu->data.gyro_z - imu->gyro_bias_z) * dt;

    /* 自动 wrap 到 [-PI, PI] */
    while (imu->data.yaw > 3.14159265358979323846f)
    {
        imu->data.yaw -= 6.28318530717958647692f;
    }
    while (imu->data.yaw < -3.14159265358979323846f)
    {
        imu->data.yaw += 6.28318530717958647692f;
    }
}

/**
 * @brief 陀螺仪零偏校准 (上电静止时调用)
 *
 * 连续采样 gyro_z, 取平均值作为零偏。
 * 采样期间底盘必须保持静止!
 *
 * @param imu      IMU 句柄指针
 * @param samples  采样次数 (建议 200~500)
 */
void MPU6500_CalibrateGyro(MPU6500_t *imu, int samples)
{
    int    i;
    float  sum = 0.0f;

    if (imu == NULL || samples <= 0)
    {
        return;
    }

    /* 预热: 先读 100 次不做记录, 等待 MEMS 起振稳定 (约 1 秒) */
    for (i = 0; i < 100; i++)
    {
        MPU6500_Read_Gyro(imu);
        HAL_Delay(10);
    }

    /* 正式采样 */
    for (i = 0; i < samples; i++)
    {
        MPU6500_Read_Gyro(imu);
        sum += imu->data.gyro_z;
        HAL_Delay(10);  /* 10ms 间隔, 与主循环一致 */
    }

    imu->gyro_bias_z = sum / (float)samples;
}
