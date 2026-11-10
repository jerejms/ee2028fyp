#include "main.h"
#include "sensors.h"

#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
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
