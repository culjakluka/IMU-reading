/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsox_reg.h"
#include "lis3mdl_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;

stmdev_ctx_t lsm6dsox_ctx;
stmdev_ctx_t lis3mdl_ctx;

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float mag_x, mag_y, mag_z;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
// I2C read and write functions
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
		uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

// Initialization functions
void LSM6DSOX_Init(void);
void LIS3MDL_Init(void);

// Sensor data read functions
void LSM6DSOX_ReadAccel(float *x, float *y, float *z);
void LSM6DSOX_ReadGyro(float *x, float *y, float *z);
void LIS3MDL_ReadMag(float *x, float *y, float *z);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	LSM6DSOX_Init();
	LIS3MDL_Init();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		// Variables for sensor readings


		// Read sensor data
		LSM6DSOX_ReadAccel(&accel_x, &accel_y, &accel_z);
		LSM6DSOX_ReadGyro(&gyro_x, &gyro_y, &gyro_z);
		LIS3MDL_ReadMag(&mag_x, &mag_y, &mag_z);

		// Debug: Print sensor data over UART
		printf("Accel: X=%.2f Y=%.2f Z=%.2f\n", accel_x, accel_y, accel_z);
		printf("Gyro: X=%.2f Y=%.2f Z=%.2f\n", gyro_x, gyro_y, gyro_z);
		printf("Mag:  X=%.2f Y=%.2f Z=%.2f\n", mag_x, mag_y, mag_z);

		HAL_Delay(500);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void LSM6DSOX_Init(void) {
    uint8_t who_am_i;

    // Initialize context
    lsm6dsox_ctx.write_reg = platform_write;
    lsm6dsox_ctx.read_reg = platform_read;
    lsm6dsox_ctx.handle = &hi2c1;

    // Check WHO_AM_I register
    lsm6dsox_device_id_get(&lsm6dsox_ctx, &who_am_i);
    if (who_am_i != LSM6DSOX_ID) {
        printf("LSM6DSOX not detected! WHO_AM_I=0x%02X\n", who_am_i);
        while (1);
    }

    // Reset device
    lsm6dsox_reset_set(&lsm6dsox_ctx, PROPERTY_ENABLE);
    uint8_t reset;
    do {
        lsm6dsox_reset_get(&lsm6dsox_ctx, &reset);
    } while (reset);

    // Enable Block Data Update
    lsm6dsox_block_data_update_set(&lsm6dsox_ctx, PROPERTY_ENABLE);

    // Configure accelerometer (2g, 416 Hz)
    lsm6dsox_xl_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_XL_ODR_417Hz);
    lsm6dsox_xl_full_scale_set(&lsm6dsox_ctx, LSM6DSOX_2g);

    // Configure gyroscope (2000 dps, 416 Hz)
    lsm6dsox_gy_data_rate_set(&lsm6dsox_ctx, LSM6DSOX_GY_ODR_417Hz);
    lsm6dsox_gy_full_scale_set(&lsm6dsox_ctx, LSM6DSOX_2000dps);

    printf("LSM6DSOX initialized.\n");
}

/**
 * @brief Initialize the LIS3MDL sensor.
 */
void LIS3MDL_Init(void) {
    uint8_t who_am_i;

    // Initialize context
    lis3mdl_ctx.write_reg = platform_write;
    lis3mdl_ctx.read_reg = platform_read;
    lis3mdl_ctx.handle = &hi2c1;

    // Check WHO_AM_I register
    lis3mdl_device_id_get(&lis3mdl_ctx, &who_am_i);
    if (who_am_i != LIS3MDL_ID) {
        printf("LIS3MDL not detected! WHO_AM_I=0x%02X\n", who_am_i);
        while (1);
    }

    // Reset device
    lis3mdl_reset_set(&lis3mdl_ctx, PROPERTY_ENABLE);
    uint8_t reset;
    do {
        lis3mdl_reset_get(&lis3mdl_ctx, &reset);
    } while (reset);

    // Configure magnetometer (Ultra-high performance, 10 Hz)
    lis3mdl_block_data_update_set(&lis3mdl_ctx, PROPERTY_ENABLE);
    lis3mdl_data_rate_set(&lis3mdl_ctx, LIS3MDL_UHP_10Hz);
    lis3mdl_full_scale_set(&lis3mdl_ctx, LIS3MDL_4_GAUSS);
    lis3mdl_operating_mode_set(&lis3mdl_ctx, LIS3MDL_CONTINUOUS_MODE);

    printf("LIS3MDL initialized.\n");
}
/**
 * @brief Read accelerometer data.
 */
void LSM6DSOX_ReadAccel(float *x, float *y, float *z) {
    int16_t data_raw[3];
    lsm6dsox_acceleration_raw_get(&lsm6dsox_ctx, data_raw);
    *x = lsm6dsox_from_fs2_to_mg(data_raw[0]) / 1000.0f;
    *y = lsm6dsox_from_fs2_to_mg(data_raw[1]) / 1000.0f;
    *z = lsm6dsox_from_fs2_to_mg(data_raw[2]) / 1000.0f;
}

/**
 * @brief Read gyroscope data.
 */
void LSM6DSOX_ReadGyro(float *x, float *y, float *z) {
    int16_t data_raw[3];
    lsm6dsox_angular_rate_raw_get(&lsm6dsox_ctx, data_raw);
    *x = lsm6dsox_from_fs2000_to_mdps(data_raw[0]) / 1000.0f;
    *y = lsm6dsox_from_fs2000_to_mdps(data_raw[1]) / 1000.0f;
    *z = lsm6dsox_from_fs2000_to_mdps(data_raw[2]) / 1000.0f;
}

/**
 * @brief Read magnetometer data.
 */
void LIS3MDL_ReadMag(float *x, float *y, float *z) {
    int16_t data_raw[3];
    lis3mdl_magnetic_raw_get(&lis3mdl_ctx, data_raw);
    *x = lis3mdl_from_fs4_to_gauss(data_raw[0]);
    *y = lis3mdl_from_fs4_to_gauss(data_raw[1]);
    *z = lis3mdl_from_fs4_to_gauss(data_raw[2]);
}

/**
 * @brief Platform-specific write function.
 */
int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    return HAL_I2C_Mem_Write((I2C_HandleTypeDef *)handle, LSM6DSOX_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)bufp, len, HAL_MAX_DELAY);
}

/**
 * @brief Platform-specific read function.
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    return HAL_I2C_Mem_Read((I2C_HandleTypeDef *)handle, LSM6DSOX_I2C_ADD_H, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, HAL_MAX_DELAY);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
