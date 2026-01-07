/* USER CODE BEGIN Header */
/**
 * @brief          : Micromouse Speed Ramp & Smart Turn Test
 * @hardware       : TB6612FNG (TIM5), MPU6050 (I2C1), Encoders (TIM1/TIM2)
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>

/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define GYRO_ZOUT_H_REG 0x47
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
/* USER CODE END PD */

/* USER CODE BEGIN PV */
// --- IMU Variables ---
volatile float Gyro_Z_Val;
volatile float Gyro_Error_Z = 0.0;
volatile float Yaw = 0.0;
uint32_t prev_time, curr_time;
float dt;
int mode = 0;

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
void MPU6050_Update_Yaw(void); // Helper to keep Yaw updated

// --- MOVEMENT FUNCTIONS ---
void Motor_Drive(int left_speed, int right_speed);
void Move_Forward(int speed, int duration_ms);
void Turn_Right(float degrees);
void Turn_Left(float degrees);
/* USER CODE END PFP */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM5_Init();

    /* USER CODE BEGIN 2 */
    // 1. Hardware Init
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    HAL_Delay(100);
    MPU6050_Init();

    // 2. Calibrate (Keep Still!)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
    MPU6050_Calibrate();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF

    prev_time = HAL_GetTick();
    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // --- TEST 1: SPEED RAMP (Up and Down) ---

        // Accelerate
        for (int speed = 0; speed <= 500; speed += 1)
        {
            mode = 1;
            MPU6050_Update_Yaw(); // Always update Yaw
            Motor_Drive(speed, speed);
            HAL_Delay(10); // Ramp speed roughly every 10ms
        }

        // Decelerate
        for (int speed = 500; speed >= 0; speed -= 1)
        {
            mode = 2;
            MPU6050_Update_Yaw();
            Motor_Drive(speed, speed);
            HAL_Delay(10);
        }

        HAL_Delay(1000); // Wait 1 second before turns

        // --- TEST 2: SMART TURNS ---

        // Turn Right 90 degrees
        mode = 3;
        Turn_Right(90);
        HAL_Delay(500);

        // Turn Left 90 degrees (Should return to roughly original heading)
        mode = 4;
        Turn_Left(90);
        HAL_Delay(1000);
    }
    /* USER CODE END WHILE */
}

/* USER CODE BEGIN 4 */

// --- IMU FUNCTIONS ---

void MPU6050_Update_Yaw(void)
{
    MPU6050_Read_Gyro();
    curr_time = HAL_GetTick();
    dt = (curr_time - prev_time) / 1000.0f;
    prev_time = curr_time;
    Yaw += (Gyro_Z_Val - Gyro_Error_Z) * dt;
}

void MPU6050_Read_Gyro(void)
{
    uint8_t Rec_Data[2];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 100);
    int16_t temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    Gyro_Z_Val = (float)temp / 131.0;
}

// --- MOVEMENT FUNCTIONS ---

void Motor_Drive(int left_speed, int right_speed)
{
    // LEFT MOTOR
    if (left_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        left_speed = -left_speed;
    }

    // RIGHT MOTOR
    if (right_speed >= 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
        right_speed = -right_speed;
    }

    // Cap Speed
    if (left_speed > 500)
        left_speed = 500;
    if (right_speed > 500)
        right_speed = 500;

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, right_speed);
}

void Move_Forward(int speed, int duration_ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms)
    {
        MPU6050_Update_Yaw(); // Keep tracking angle while moving
        Motor_Drive(speed, speed);
    }
    Motor_Drive(0, 0);
}

void Turn_Right(float degrees)
{
    float start_angle = Yaw;
    // Right turn usually decreases Yaw (Clockwise is negative Z-axis rotation)
    // Target is "Current - 90"
    float target_angle = start_angle - degrees;

    // Spin until we pass the target
    while (Yaw > target_angle)
    {
        MPU6050_Update_Yaw();   // CRITICAL: Must update Yaw inside loop
        Motor_Drive(300, -300); // Spin Right (Left Fwd, Right Rev)
        HAL_Delay(1);           // Stability delay
    }
    Motor_Drive(0, 0); // Hard Stop
}

void Turn_Left(float degrees)
{
    float start_angle = Yaw;
    // Left turn increases Yaw
    float target_angle = start_angle + degrees;

    while (Yaw < target_angle)
    {
        MPU6050_Update_Yaw();
        Motor_Drive(-300, 300); // Spin Left (Left Rev, Right Fwd)
        HAL_Delay(1);
    }
    Motor_Drive(0, 0);
}

// --- INIT & CALIBRATION ---

void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t Data;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if (check == 0x68)
    {
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 100);
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 100);
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 100);
    }
}

void MPU6050_Calibrate(void)
{
    float sum = 0;
    uint8_t Rec_Data[2];
    for (int i = 0; i < 1000; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_ZOUT_H_REG, 1, Rec_Data, 2, 100);
        int16_t temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        sum += (float)temp / 131.0;
        HAL_Delay(1);
    }
    Gyro_Error_Z = sum / 1000.0;
}

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
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 200;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
/* USER CODE END 4 */
