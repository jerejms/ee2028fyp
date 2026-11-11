#ifndef SENSORS_H
#define SENSORS_H

typedef struct{
    float temperature;       // degC
    float pressure;          // hPa
    float humidity;          // %RH
    float acceleration[3];   // m/s^2
    float gyroscope[3];      // deg/s
    float magnetic[3];       // uT
} SensorReadings;

void Sensors_Init(void);
void Sensors_Read(SensorReadings *readings);
float Sensors_ReadAccelerationZ(void);
void Sensors_ResetGravityFilter(void);
float Sensors_ReadGravityZ(float dt);

#endif
