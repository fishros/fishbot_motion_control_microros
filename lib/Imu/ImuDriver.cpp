#include "ImuDriver.h"

/**
 * @brief Construct a new ImuDriver object
 *
 * @param mpu 传递MPU6050的驱动引用
 */
ImuDriver::ImuDriver(MPU6050 &mpu)
{
    mpu_ = &mpu;
}

/**
 * @brief 初始化MPU6050
 *
 * @param sda 引脚编号，18
 * @param scl 引脚编号，19
 * @return true
 * @return false
 */
bool ImuDriver::begin(int sda, int scl)
{
    Wire.begin(sda, scl);
    byte status = mpu_->begin();
    if (status != 0)
    {
        return imu_enable_;
    } // stop everything if could not connect to MPU6050

    // Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu_->calcOffsets(); // gyro and accelero
    imu_enable_ = true;
    return imu_enable_;
}

float ImuDriver::getYaw()
{
    return mpu_->getAngleZ();
}

bool ImuDriver::isEnable()
{
    return imu_enable_;
}

/**
 * @brief 更新IMU数据，两次调用该函数时，需要有一定的延时
 *
 */
void ImuDriver::update()
{
    if (imu_enable_)
    {
        mpu_->update();
    }
}
/**
 * @brief 欧拉角转四元数
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @param q
 */
void ImuDriver::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_imu_t &q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

/**
 * @brief Get the ImuDriver Data object
 *
 * @param imu
 */
void ImuDriver::getImuDriverData(imu_t &imu)
{
    imu.angular_velocity.x = mpu_->getGyroX();
    imu.angular_velocity.y = mpu_->getGyroY();
    imu.angular_velocity.z = mpu_->getGyroZ();

    imu.linear_acceleration.x = mpu_->getAccX();
    imu.linear_acceleration.y = mpu_->getAccY();
    imu.linear_acceleration.z = mpu_->getAccZ();

    imu.orientation_euler.x = (mpu_->getAngleX()/RAD_2_DEG);
    imu.orientation_euler.y = (mpu_->getAngleY()/RAD_2_DEG);
    imu.orientation_euler.z = (mpu_->getAngleZ()/RAD_2_DEG);

    Euler2Quaternion(imu.orientation_euler.x, imu.orientation_euler.y, imu.orientation_euler.z,
                     imu.orientation);
}