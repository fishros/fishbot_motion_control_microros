#ifndef __IMU_H__
#define __IMU_H__
#include "Wire.h"
#include "MPU6050_light.h"

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_imu_t;

typedef struct
{
    float x;
    float y;
    float z;
} vector_3d_t;

typedef struct
{
    quaternion_imu_t orientation;
    vector_3d_t orientation_euler;
    vector_3d_t angular_velocity;
    vector_3d_t linear_acceleration;
} imu_t;

class ImuDriver
{
private:
    MPU6050 *mpu_;
    bool imu_enable_{false};

public:
    /**
     * @brief Construct a new ImuDriver object
     *
     * @param mpu 传递MPU6050的驱动引用
     */
    ImuDriver(MPU6050 &mpu);

    ~ImuDriver() = default;

    /**
     * @brief 初始化MPU6050
     *
     * @param sda 引脚编号，18
     * @param scl 引脚编号，19
     * @return true
     * @return false
     */
    bool begin(int sda, int scl);

    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool isEnable();

    /**
     * @brief Get the Yaw object
     * 
     * @return float 
     */
    float getYaw();

    /**
     * @brief 更新IMU数据，两次调用该函数时，需要有一定的延时
     *
     */
    void update();

    /**
     * @brief 欧拉角转四元数
     *
     * @param roll
     * @param pitch
     * @param yaw
     * @param q
     */
    static void Euler2Quaternion(float roll, float pitch, float yaw, quaternion_imu_t &q);

    /**
     * @brief Get the ImuDriver Data object
     *
     * @param imu
     */
    void getImuDriverData(imu_t &imu);
};

#endif