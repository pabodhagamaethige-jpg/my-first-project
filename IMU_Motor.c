/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Hardware Test - Motors (TIM5), Encoders (TIM1/TIM2), IMU (I2C1)
 * @target         : STM32F411 "Black Pill"
 * @hardware       : TB6612FNG, N20 Motors, MPU6050
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define GYRO_ZOUT_H_REG 0x47
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// --- IMU Variables ---
volatile float Gyro_Z_Val;
volatile float Gyro_Error_Z = 0.0;
volatile float Yaw = 0.0;
uint32_t prev_time, curr_time;
float dt;

// --- Encoder Variables ---
volatile int16_t left_count = 0;
volatile int16_t right_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Calibrate(void);
void MPU6050_Read_Gyro(void);
void Motor_Drive(int left_speed, int right_speed); // Speed: -1000 to 1000
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
    MX_TIM1_Init(); // Left Encoder
    MX_TIM2_Init(); // Right Encoder (PA5/PB3)
    MX_TIM5_Init(); // PWM Motors (PA0/PA1)
    /* USER CODE BEGIN 2 */

    // 1. Start Hardware
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    // 2. IMU Setup
    HAL_Delay(100);
    MPU6050_Init();

    // 3. Calibrate Gyro (LED ON during calibration)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    MPU6050_Calibrate();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF when done

    prev_time = HAL_GetTick();

    // 4. MOTOR TEST: Spin Forward for 2 Seconds
    // Values are 0 to 1000. 500 = 50% Speed.
    Motor_Drive(500, 500);
    HAL_Delay(2000);
    Motor_Drive(0, 0); // Stop

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // --- READ GYRO ---
        MPU6050_Read_Gyro();

        // Calculate Yaw (Angle)
        curr_time = HAL_GetTick();
        dt = (curr_time - prev_time) / 1000.0f;
        prev_time = curr_time;
        Yaw += (Gyro_Z_Val - Gyro_Error_Z) * dt;

        // --- READ ENCODERS ---
        // These numbers should change when you rotate the wheels by hand
        left_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        right_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);

        // --- DEBUGGING ---
        // Open "Live Expressions" in debugger and look at:
        // 1. Yaw (Should change when you rotate the robot)
        // 2. left_count (Should change when left wheel turns)
        // 3. right_count (Should change when right wheel turns)

        HAL_Delay(10);
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;            // 25MHz Crystal -> /25 = 1MHz
    RCC_OscInitStruct.PLL.PLLN = 200;           // 1MHz * 200 = 200MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 200MHz / 2 = 100MHz
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

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

// Simple Motor Control Function
void Motor_Drive(int left_speed, int right_speed)
{
    // LEFT MOTOR
    if (left_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);   // AIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // AIN2
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        left_speed = -left_speed; // Make positive for PWM
    }

    // RIGHT MOTOR
    if (right_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // BIN1
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);  // BIN2
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        right_speed = -right_speed;
    }

    // Cap speed at 1000
    if (left_speed > 1000)
        left_speed = 1000;
    if (right_speed > 1000)
        right_speed = 1000;

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, right_speed);
}

void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;

    // Check ID
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if (check == 0x68)
    {
        // Wake Up
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
        // Data Rate 1KHz
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
        // Gyro Range +/- 250
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
    }
}

void MPU6050_Calibrate(void)
{
    float sum = 0;
    uint8_t Rec_Data[2];

    // Read 1000 samples to find the offset
    for (int i = 0; i < 1000; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 1000);
        int16_t temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        sum += (float)temp / 131.0;
        HAL_Delay(1);
    }
    Gyro_Error_Z = sum / 1000.0;
}

void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[2];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 1000);
    int16_t temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Z_Val = (float)temp / 131.0;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
