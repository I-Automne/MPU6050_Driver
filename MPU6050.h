//===========================================================
// MPU6050.h配置
//===========================================================

// IMU极性矩阵，X、Y、Z的对角分别是极性：
// 例如1,1,-1就是XY方向与板丝印方向一致，Z方向倒置
static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define ERROR_MPU_INIT      -1
#define ERROR_SET_SENSOR    -2
#define ERROR_CONFIG_FIFO   -3
#define ERROR_SET_RATE      -4
#define ERROR_LOAD_MOTION_DRIVER    -5
#define ERROR_SET_ORIENTATION       -6
#define ERROR_ENABLE_FEATURE        -7
#define ERROR_SET_FIFO_RATE         -8
#define ERROR_SELF_TEST             -9
#define ERROR_DMP_STATE             -10

#define DEFAULT_MPU_HZ  100
#define Q30  1073741824.0f

int MPU6050_DMP_init(void);

//===========================================================
// IIC配置
//===========================================================

// 使用软件IIC或者硬件IIC：
// 使用IIC，都需要在inv_mpu.c的开头进行配置与更改
// 所有与IIC的接口都只在这一处地方，也就是重写MPU库内部的IIC函数
#define USE_SOFTWARE_IIC     1
#define USE_HARDWARE_IIC     0

//===========================================================
// MPU6050读取数据函数
//===========================================================
int MPU6050_DMP_Get6Axis_RawData(short *accel, short *gyro);
int MPU6050_DMP_Get6Axis(float *accel, float *gyro);
int MPU6050_DMP_GetRPY(float *roll, float *pitch, float *yaw);

#endif /* INC_MPU6050_H_ */
