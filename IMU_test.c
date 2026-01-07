/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - MPU6050 Yaw Calculation
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usb_device.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define GYRO_ZOUT_H_REG 0x47 // Added Gyro Z Register
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// --- MPU6050 Variables ---
volatile uint8_t mpu_id = 0;

// Accelerometer Data
volatile int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
volatile float Ax, Ay, Az;

// Gyroscope Data
volatile int16_t Gyro_Z_RAW;
volatile float Gz;
volatile float Gyro_Error_Z = 0.0; // Stores the average error
volatile float Yaw = 0.0;          // Total accumulated Angle

// Timer variables for integration
uint32_t prev_time;
uint32_t curr_time;
float dt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Calibrate_Gyro(void); // New function
void MPU6050_Read_All(void);       // Updated function
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
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
    MX_USB_DEVICE_Init();

    /* USER CODE BEGIN 2 */
    HAL_Delay(500); // Wait for sensor to power up

    MPU6050_Init();

    // *** CRITICAL STEP: Keep board still here! ***
    MPU6050_Calibrate_Gyro();

    prev_time = HAL_GetTick(); // Start timer
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // 1. Toggle LED
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        // 2. Read Sensors
        MPU6050_Read_All();

        // 3. Calculate Time Difference (dt)
        //    We need this to know how much time passed since last loop
        curr_time = HAL_GetTick();
        dt = (curr_time - prev_time) / 1000.0f; // Convert ms to seconds
        prev_time = curr_time;

        // 4. Calculate Yaw (Integration)
        //    Angle = Angle + (Speed * Time)
        //    We subtract the Error we found during startup
        Yaw = Yaw + (Gz - Gyro_Error_Z) * dt;

        // Debug in Live Expressions: Watch 'Yaw'

        HAL_Delay(10); // Small delay
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        // Wake up
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

        // Data Rate 1KHz
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

        // Accel Config ±2g
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

        // Gyro Config ±250 °/s
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    }
}

void MPU6050_Calibrate_Gyro(void)
{
    // Take 2000 readings while the board is still to find the error offset
    float sum = 0;
    for (int i = 0; i < 2000; i++)
    {
        uint8_t Rec_Data[2];
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 1000);
        int16_t Temp_Gyro_Z = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);

        // Divide by 131.0 to convert Raw to Deg/s (for ±250 range)
        sum += (float)Temp_Gyro_Z / 131.0;
        HAL_Delay(1);
    }
    Gyro_Error_Z = sum / 2000.0;
}

void MPU6050_Read_All(void)
{
    uint8_t Rec_Data[6];

    // --- Read Accel ---
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    Ax = Accel_X_RAW / 16384.0;
    Ay = Accel_Y_RAW / 16384.0;
    Az = Accel_Z_RAW / 16384.0;

    // --- Read Gyro Z Only ---
    uint8_t Gyro_Data[2];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Gyro_Data, 2, 1000);
    Gyro_Z_RAW = (int16_t)(Gyro_Data[0] << 8 | Gyro_Data[1]);

    // Convert to degrees per second
    Gz = Gyro_Z_RAW / 131.0;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
