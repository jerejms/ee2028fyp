  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) EE2028 Teaching Team
  ******************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "stdio.h"

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

int main(void)
{
	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Peripheral initializations using BSP functions */
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();
	BSP_MAGNETO_Init();
	BSP_TSENSOR_Init();
	BSP_PSENSOR_Init();
	BSP_HSENSOR_Init();

	while (1)
	{
		float accel_data[3];
		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings.
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
		// the function above returns 16 bit integers which are acceleration in mg (9.8/1000 m/s^2).
		// Converting to float to print the actual acceleration.
		accel_data[0] = (float)accel_data_i16[0] * (9.8/1000.0f);
		accel_data[1] = (float)accel_data_i16[1] * (9.8/1000.0f);
		accel_data[2] = (float)accel_data_i16[2] * (9.8/1000.0f);

		float temp_data = BSP_TSENSOR_ReadTemp();		// read temperature sensor

		float pressure_data = BSP_PSENSOR_ReadPressure();

		float humidity_data = BSP_HSENSOR_ReadHumidity();

		float gyro_data[3];
		float gyro_data_float[3] = { 0 };
		BSP_GYRO_GetXYZ(gyro_data_float);
		gyro_data[0] = gyro_data_float[0];
		gyro_data[1] = gyro_data_float[1];
		gyro_data[2] = gyro_data_float[2];

		//conversion blackmagic

		float magneto_data[3];
		int16_t magneto_data_i16[3] = { 0 };
		BSP_MAGNETO_GetXYZ(magneto_data_i16);
		magneto_data[0] = (float)magneto_data_i16[0]/10;
		magneto_data[1] = (float)magneto_data_i16[1]/10;
		magneto_data[2] = (float)magneto_data_i16[2]/10;



		//conversion blackmagic

		printf("T: %.2f degC | P: %.2f hPa | H: %.2f %%RH\r\n"
				"A: X=%.2f Y=%.2f Z=%.2f m/s^2\r\n"
				"G: X=%.2f Y=%.2f Z=%.2f deg/s\r\n"
				"M: X=%.2f Y=%.2f Z=%.2f uT\r\n\r\n",
				temp_data,
				pressure_data,
				humidity_data,
				accel_data[0], accel_data[1], accel_data[2],
				gyro_data[0], gyro_data[1], gyro_data[2],
				magneto_data[0], magneto_data[1], magneto_data[2]);
		HAL_Delay(500);	// read once a ~ half second.
	}

}
