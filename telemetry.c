#include "telemetry.h"
#include "uart.h"
#include <stdio.h>

#define TEMP_MIN 15.0f
#define TEMP_MAX 40.0f
#define PRESSURE_MIN 950.0f
#define PRESSURE_MAX 1050.0f
#define HUMIDITY_MIN 30.0f
#define HUMIDITY_MAX 80.0f
#define ACCEL_MAX 15.0f
#define GYRO_MAX 100.0f
#define MAGNETIC_MAX 1000.0f

// return 0 if safe within range, else return 1 (warning)
static int CheckRange(const char *name, float value, float minimum, float maximum, const char *unit){
    char message[128];

    if (value < minimum){
        snprintf(message, sizeof(message),
                 "%s: %.2f %s, below threshold of %.2f %s\r\n",
                 name, value, unit, minimum, unit);
    }
    else if (value > maximum){
        snprintf(message, sizeof(message),
                 "%s: %.2f %s, exceeds threshold of %.2f %s\r\n",
                 name, value, unit, maximum, unit);
    }
    else{
        return 0;
    }

    UART_SendText(message);
    return 1;
}

void Telemetry_Send(const SensorReadings *readings){
    int violations = 0;
    // check temp, pressure and humidity
    violations += CheckRange(
        "T",
        readings->temperature,
        TEMP_MIN,
        TEMP_MAX,
        "degC"
    );
    violations += CheckRange(
        "P",
        readings->pressure,
        PRESSURE_MIN,
        PRESSURE_MAX,
        "hPa"
    );
    violations += CheckRange(
        "H",
        readings->humidity,
        HUMIDITY_MIN,
        HUMIDITY_MAX,
        "%RH"
    );

    //all 3 axes for accel, gyro, and magnetic field
    const char *accelNames[3] = {"A_X", "A_Y", "A_Z"};
    const char *gyroNames[3] = {"G_X", "G_Y", "G_Z"};
    const char *magneticNames[3] = {"M_X", "M_Y", "M_Z"};

    // Check all three axes of each motion/field sensor.
    for (int i = 0; i < 3; i++)
    {
        violations += CheckRange(
            accelNames[i],
            readings->acceleration[i],
            -ACCEL_MAX,
            ACCEL_MAX,
            "m/s^2"
        );

        violations += CheckRange(
            gyroNames[i],
            readings->gyroscope[i],
            -GYRO_MAX,
            GYRO_MAX,
            "deg/s"
        );

        violations += CheckRange(
            magneticNames[i],
            readings->magnetic[i],
            -MAGNETIC_MAX,
            MAGNETIC_MAX,
            "uT"
        );
    }

    //if got violation, return
    if (violations > 0){
        return;
    }

    //else, send normal telem
    char message[128];
    snprintf(message, sizeof(message),
             "T: %.2f degC | P: %.2f hPa | H: %.2f %%RH\r\n",
             readings->temperature,
             readings->pressure,
             readings->humidity);
    UART_SendText(message);

    snprintf(message, sizeof(message),
             "A: X=%.2f Y=%.2f Z=%.2f m/s^2\r\n",
             readings->acceleration[0],
             readings->acceleration[1],
             readings->acceleration[2]);
    UART_SendText(message);

    snprintf(message, sizeof(message),
             "G: X=%.2f Y=%.2f Z=%.2f deg/s\r\n",
             readings->gyroscope[0],
             readings->gyroscope[1],
             readings->gyroscope[2]);
    UART_SendText(message);

    snprintf(message, sizeof(message),
             "M: X=%.2f Y=%.2f Z=%.2f uT\r\n\r\n",
             readings->magnetic[0],
             readings->magnetic[1],
             readings->magnetic[2]);
    UART_SendText(message);
}
