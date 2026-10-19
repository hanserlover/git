/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "motor.h"
#include "encoder.h"
#include "sensor.h"
#include "key.h"
#include "oled.h"
#include "imu601.h"
#include "move.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_SPEED 1200 
#define ONE_METER_RUN_BUILD 1U
#define ONE_METER_SPEED 900
#define ONE_METER_TIMEOUT_MS 15000U
#define ONE_METER_TARGET_LF 7851L
#define ONE_METER_TARGET_RF 7905L
#define ONE_METER_TARGET_LR 7825L
#define ONE_METER_TARGET_RR 7562L
#define OLED_TEXT_SIZE 12U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t tick_task_10ms = 0;
uint32_t tick_task_100ms = 0;
uint8_t robot_state = 0;
uint8_t work_mode = 0;

/* 一米运行状态和最终计数，保留为全局变量便于调试器回读。 */
volatile uint8_t g_one_meter_state = 0;
volatile int32_t g_one_meter_counts[ENCODER_NUM] = {0, 0, 0, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t OneMeter_Abs32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static uint8_t CenterButton_Pressed(void)
{
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) != GPIO_PIN_RESET) {
    return 0U;
  }

  HAL_Delay(25);
  return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1U : 0U;
}

static void CenterButton_WaitRelease(void)
{
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
    HAL_Delay(10);
  }
  HAL_Delay(25);
}

static void OneMeter_ReadCounts(void)
{
  g_one_meter_counts[ENCODER_LF] = OneMeter_Abs32(Encoder_GetCount(ENCODER_LF));
  g_one_meter_counts[ENCODER_RF] = OneMeter_Abs32(Encoder_GetCount(ENCODER_RF));
  g_one_meter_counts[ENCODER_LR] = OneMeter_Abs32(Encoder_GetCount(ENCODER_LR));
  g_one_meter_counts[ENCODER_RR] = OneMeter_Abs32(Encoder_GetCount(ENCODER_RR));
}

static void OneMeter_ShowReady(void)
{
  OLED_Clear();
  OLED_ShowString(0, 0, "ONE METER RUN", OLED_TEXT_SIZE);
  OLED_ShowString(0, 16, "DISTANCE: 100CM", OLED_TEXT_SIZE);
  OLED_ShowString(0, 32, "PRESS CENTER", OLED_TEXT_SIZE);
  OLED_ShowString(0, 48, "CENTER=E-STOP", OLED_TEXT_SIZE);
  OLED_Refresh();
}

static void OneMeter_ShowRunning(void)
{
  char line[24];

  OLED_Clear();
  OLED_ShowString(0, 0, "RUNNING 100CM", OLED_TEXT_SIZE);
  sprintf(line, "LF:%ld RF:%ld",
          (long)g_one_meter_counts[ENCODER_LF],
          (long)g_one_meter_counts[ENCODER_RF]);
  OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);
  sprintf(line, "LR:%ld RR:%ld",
          (long)g_one_meter_counts[ENCODER_LR],
          (long)g_one_meter_counts[ENCODER_RR]);
  OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);
  OLED_ShowString(0, 48, "CENTER=E-STOP", OLED_TEXT_SIZE);
  OLED_Refresh();
}

static void OneMeter_ShowResult(uint8_t result)
{
  char line[24];

  OLED_Clear();
  if (result == 2U) {
    OLED_ShowString(0, 0, "1M COMPLETE", OLED_TEXT_SIZE);
  } else if (result == 3U) {
    OLED_ShowString(0, 0, "EMERGENCY STOP", OLED_TEXT_SIZE);
  } else {
    OLED_ShowString(0, 0, "TIMEOUT/ENC ERROR", OLED_TEXT_SIZE);
  }
  sprintf(line, "LF:%ld RF:%ld",
          (long)g_one_meter_counts[ENCODER_LF],
          (long)g_one_meter_counts[ENCODER_RF]);
  OLED_ShowString(0, 16, line, OLED_TEXT_SIZE);
  sprintf(line, "LR:%ld RR:%ld",
          (long)g_one_meter_counts[ENCODER_LR],
          (long)g_one_meter_counts[ENCODER_RR]);
  OLED_ShowString(0, 32, line, OLED_TEXT_SIZE);
  OLED_ShowString(0, 48, "CENTER=RUN AGAIN", OLED_TEXT_SIZE);
  OLED_Refresh();
}

static uint8_t OneMeter_RunOnce(void)
{
  uint32_t start_tick;
  uint32_t last_refresh;
  uint8_t lf_done = 0U;
  uint8_t rf_done = 0U;
  uint8_t lr_done = 0U;
  uint8_t rr_done = 0U;

  Encoder_ClearAll();
  OneMeter_ReadCounts();
  g_one_meter_state = 1U;
  Move_Forward(ONE_METER_SPEED);
  start_tick = HAL_GetTick();
  last_refresh = start_tick;

  while (1) {
    OneMeter_ReadCounts();

    if (!lf_done && g_one_meter_counts[ENCODER_LF] >= ONE_METER_TARGET_LF) {
      Motor_Stop(MOTOR_LF);
      lf_done = 1U;
    }
    if (!rf_done && g_one_meter_counts[ENCODER_RF] >= ONE_METER_TARGET_RF) {
      Motor_Stop(MOTOR_RF);
      rf_done = 1U;
    }
    if (!lr_done && g_one_meter_counts[ENCODER_LR] >= ONE_METER_TARGET_LR) {
      Motor_Stop(MOTOR_LR);
      lr_done = 1U;
    }
    if (!rr_done && g_one_meter_counts[ENCODER_RR] >= ONE_METER_TARGET_RR) {
      Motor_Stop(MOTOR_RR);
      rr_done = 1U;
    }

    if (lf_done && rf_done && lr_done && rr_done) {
      Move_Stop();
      OneMeter_ReadCounts();
      g_one_meter_state = 2U;
      return 2U;
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
      Move_Stop();
      OneMeter_ReadCounts();
      g_one_meter_state = 3U;
      return 3U;
    }

    if ((HAL_GetTick() - start_tick) >= ONE_METER_TIMEOUT_MS) {
      Move_Stop();
      OneMeter_ReadCounts();
      g_one_meter_state = 4U;
      return 4U;
    }

    if ((HAL_GetTick() - last_refresh) >= 100U) {
      last_refresh = HAL_GetTick();
      OneMeter_ShowRunning();
    }
    HAL_Delay(2);
  }
}

static void OneMeter_ModeRun(void)
{
  uint8_t result;

  Move_Stop();
  g_one_meter_state = 0U;
  OneMeter_ShowReady();
  CenterButton_WaitRelease();

  while (1) {
    while (!CenterButton_Pressed()) {
      HAL_Delay(10);
    }
    CenterButton_WaitRelease();

    result = OneMeter_RunOnce();
    OneMeter_ShowResult(result);
    CenterButton_WaitRelease();
  }
}
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
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  Motor_Init();
  Encoder_Init();
  Sensor_Init();
  Key_Init();
  OLED_Init();
  IMU601_Init();

#if ONE_METER_RUN_BUILD
  OneMeter_ModeRun();
#endif

  IMU_SoftReset_Send(); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t current_tick = HAL_GetTick(); // 获取当前系统滴答，非阻塞

    // =========================================================
    // 10ms 周期任务：传感器读取、按键扫描、电机动作
    // =========================================================
    if (current_tick - tick_task_10ms >= 10) 
    {
        tick_task_10ms = current_tick;
        
        // --- 直接读引脚状态（最暴力的测试） ---
        // 根据最新的引脚表：
        // UP    = PB13
        // LEFT  = PB14
        // DOWN  = PB15
        // RIGHT = PD2
        // CENTER= PA1
        
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) {
            // UP键按下：前进
            Move_Forward(DEFAULT_SPEED); 
        } 
        else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) {
            // DOWN键按下：后退
            Move_Backward(DEFAULT_SPEED); 
        } 
        else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
            // LEFT键按下：左平移
            Move_Left(DEFAULT_SPEED); 
        } 
        else if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET) {
            // RIGHT键按下：右平移
            Move_Right(DEFAULT_SPEED); 
        } 
        else if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
            // CENTER键按下：原地左转
            Move_RotateLeft(DEFAULT_SPEED); 
        } 
        else {
            // 没有按键按下：停止
            Move_Stop(); 
        }
        // ------------------------------------------

        // 2. 解析 IMU 串口数据
        IMU601_Process();
        
        // 3. 传感器读取 (注意最后一个枚举改为了 SENSOR_LS)
        uint8_t obs_lf = Sensor_GetState(SENSOR_LF);
        uint8_t obs_rf = Sensor_GetState(SENSOR_RF);
        uint8_t track  = Sensor_GetState(SENSOR_LS); // 对应 PA12 左斜下
        
        (void)obs_lf; (void)obs_rf; (void)track; // 防止编译报 unused 警告
        
        // 4. 编码器读取
        int32_t enc_lr = Encoder_GetCount(ENCODER_LR);
        int32_t enc_lf = Encoder_GetCount(ENCODER_LF);
        int32_t enc_rr = Encoder_GetCount(ENCODER_RR);
        int32_t enc_rf = Encoder_GetCount(ENCODER_RF);

        (void)enc_lr; (void)enc_lf; (void)enc_rr; (void)enc_rf;
    }

    // =========================================================
    // 100ms 周期任务：OLED 刷新、状态机变量打印
    // =========================================================
    if (current_tick - tick_task_100ms >= 100) 
    {
        tick_task_100ms = current_tick;
        
        char display_buf[20];
        
        // 清屏并显示当前状态机编号
        OLED_Clear();
        
        OLED_ShowString(0, 0, "State:", OLED_TEXT_SIZE);
        sprintf(display_buf, "%d", robot_state);
        OLED_ShowString(42, 0, display_buf, OLED_TEXT_SIZE); 
        
        OLED_ShowString(0, 16, "Mode:", OLED_TEXT_SIZE);
        sprintf(display_buf, "%d", work_mode);
        OLED_ShowString(42, 16, display_buf, OLED_TEXT_SIZE);
        OLED_Refresh();
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
