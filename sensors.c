#include "main.h"
#include "sensors.h"
#include <math.h>

#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"

#define DEG_TO_RAD 0.01745329252f
#define FILTER_TAU 0.5f

static float gravityX = 0.0f;
static float gravityY = 0.0f;
static float gravityZ = 1.0f;
static uint8_t gravityInitialized = 0;

void Sensors_Init(void){
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_MAGNETO_Init();
    BSP_TSENSOR_Init();
    BSP_PSENSOR_Init();
    BSP_HSENSOR_Init();
}

void Sensors_Read(SensorReadings *readings){
    int16_t accel_mg[3] = {0};
    float gyro_mdps[3] = {0};
    int16_t magneto_mgauss[3] = {0};
    BSP_ACCELERO_AccGetXYZ(accel_mg);
    BSP_GYRO_GetXYZ(gyro_mdps);
    BSP_MAGNETO_GetXYZ(magneto_mgauss);
    readings->temperature = BSP_TSENSOR_ReadTemp();
    readings->pressure = BSP_PSENSOR_ReadPressure();
    readings->humidity = BSP_HSENSOR_ReadHumidity();

    for (int i = 0; i < 3; i++){
        // Milligravity to m/s^2.
        readings->acceleration[i] = (float)accel_mg[i] * (9.8f / 1000.0f);
        // Millidegrees/s to degrees/s.
        readings->gyroscope[i] = gyro_mdps[i] / 1000.0f;
        // Milligauss to microtesla.
        readings->magnetic[i] = (float)magneto_mgauss[i] / 10.0f;
    }
}

float Sensors_ReadAccelerationZ(void){
    int16_t accel_mg[3] = {0};
    BSP_ACCELERO_AccGetXYZ(accel_mg);
    return (float)accel_mg[2] * (9.8f / 1000.0f);
}

void Sensors_ResetGravityFilter(void)
{
    gravityX = 0.0f;
    gravityY = 0.0f;
    gravityZ = 1.0f;
    gravityInitialized = 0;
}

float Sensors_ReadGravityZ(float dt)
{
    int16_t accel[3] = {0};
    float gyro[3] = {0};

    BSP_ACCELERO_AccGetXYZ(accel);
    BSP_GYRO_GetXYZ(gyro);

    // Accelerometer: milli-g
    float ax = (float)accel[0];
    float ay = (float)accel[1];
    float az = (float)accel[2];

    float magnitude = sqrtf(ax*ax + ay*ay + az*az);

    if (magnitude < 100.0f)
    {
        return gravityZ;
    }

    ax /= magnitude;
    ay /= magnitude;
    az /= magnitude;

    if (!gravityInitialized)
    {
        gravityX = ax;
        gravityY = ay;
        gravityZ = az;
        gravityInitialized = 1;
        return gravityZ;
    }

    if (dt <= 0.0f || dt > 0.2f)
    {
        return gravityZ;
    }

    // Gyroscope: mdps -> rad/s
    float wx = gyro[0] * 0.001f * DEG_TO_RAD;
    float wy = gyro[1] * 0.001f * DEG_TO_RAD;
    float wz = gyro[2] * 0.001f * DEG_TO_RAD;

    // Predict gravity vector using gyro integration
    // g_dot = -omega cross g
    float predictedX = gravityX +
        (wz * gravityY - wy * gravityZ) * dt;

    float predictedY = gravityY +
        (wx * gravityZ - wz * gravityX) * dt;

    float predictedZ = gravityZ +
        (wy * gravityX - wx * gravityY) * dt;

    // Complementary filter correction
    float alpha = FILTER_TAU / (FILTER_TAU + dt);

    float fusedX = alpha * predictedX + (1.0f-alpha)*ax;
    float fusedY = alpha * predictedY + (1.0f-alpha)*ay;
    float fusedZ = alpha * predictedZ + (1.0f-alpha)*az;

    // Normalize estimated vector
    float norm = sqrtf(
        fusedX*fusedX +
        fusedY*fusedY +
        fusedZ*fusedZ
    );

    if (norm > 0.0001f)
    {
        gravityX = fusedX / norm;
        gravityY = fusedY / norm;
        gravityZ = fusedZ / norm;
    }

    return gravityZ;
}
