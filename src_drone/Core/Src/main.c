#include "main.h"
#include "math.h"
#include "stdlib.h"
#define RAD_TO_DEG 57.295779513082320876798154814105
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define MPU6050_ADDRESS  0x68
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Accel_Offset[3]; 
float Gyro_Offset[3];
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cài d?t m?c tang và gi?i h?n PID
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.45;
float pid_i_gain_roll = 0.0003;
float pid_d_gain_roll = 0.08;
int pid_max_roll = 400;

float pid_p_gain_pitch = 1.45;
float pid_i_gain_pitch = 0.0003;
float pid_d_gain_pitch = 0.08;
int pid_max_pitch = 400;

float pid_p_gain_yaw = 1.2;
float pid_i_gain_yaw = 0.0003;
float pid_d_gain_yaw = 0.002;
int pid_max_yaw = 400;

int16_t manual_takeoff_throttle = 1500;    // Nh?p di?m c?t cánh th? công khi không phát hi?n c?t cánh t? d?ng (gi?a 1400 và 1600).
int16_t motor_idle_speed = 1100;           // Nh?p xung ga t?i thi?u c?a d?ng co khi chúng không t?i (gi?a 1000 và 1200).

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Khai báo các bi?n toàn c?c
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile int CH1, CH2, CH3, CH4, CH5, CH6;
volatile int pid_roll_setpoint_base, pid_pitch_setpoint_base;
int esc_1, esc_2, esc_3, esc_4;
int throttle, battery_voltage;
int  start, cal_int;
int temperature, count_var;
int acc_axis[4], gyro_axis[4];
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
float roll_level_adjust, pitch_level_adjust;

float acc_x, acc_y, acc_z;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t acc_x_cal, acc_y_cal;
uint32_t current_time;
uint32_t timer;

int roll_neutral = 1512;
int pitch_neutral = 1512;
int yaw_neutral = 1492;
uint8_t setpoint_locked = 0;


double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp_roll;
float pid_error_temp_pitch;
float pid_error_temp_yaw;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float angle_roll_error, angle_pitch_error;
char gyro_angles_set;

uint32_t loop_timer, error_timer, flight_mode_timer;
uint8_t level_calibration_on;
uint8_t check_byte, flip32;
uint8_t error, error_counter, error_led;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
uint8_t takeoff_detected, manual_altitude_change;
int16_t manual_throttle;
int16_t takeoff_throttle;

void calculate_pid(void);
void gyro_signalen(void);
void calibrate_gyro(void);
void gyro_setup(void);
float course_deviation(float course_b, float course_c);
void vertical_acceleration_calculations(void);
void start_stop_takeoff(void);
void error_signal(void);
void Set_ESC_Throttle() {
    TIM1->CCR1 = esc_2;  // ESC1 - PWM trên TIM1_CH1
		TIM1->CCR2 = esc_1;  // ESC2 - PWM trên TIM1_CH2
		TIM1->CCR3 = esc_4;  // ESC3 - PWM trên TIM1_CH3
		TIM1->CCR4 = esc_3;  // ESC4 - PWM trên TIM1_CH4
}
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            
}
void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (SystemCoreClock / 1000000) * us; // S? chu k? tuong ?ng v?i micro giây
    while ((DWT->CYCCNT - start) < cycles);
}
uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	DWT_Init();
	GPIOC->BSRR = (1 << 14) | (1 << (13 + 16));
	gyro_setup();
  for (count_var = 0; count_var < 1250; count_var++) {
      if (count_var % 125 == 0) {
          GPIOC->ODR ^= (1 << 14);
      }
      HAL_Delay(4);
  }
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,0);
	count_var = 0;
	calibrate_gyro();
	while (CH3 < 990 || CH3 > 1020 || CH4 < 1400) {
     error = 3;
     error_signal();
     HAL_Delay(4);
  }
	error = 0;
	GPIOC->BSRR = (1 << (14 + 16));
	//battery_voltage = (HAL_ADC_GetValue(&hadc1) + 65) * 1.2317;
  if (motor_idle_speed < 1000) motor_idle_speed = 1000;
  if (motor_idle_speed > 1200) motor_idle_speed = 1200;
	GPIOC->BSRR = (1 << 13);
	current_time = micros();

  while (1)
  {
		error_signal();
		gyro_signalen();
		gyro_roll -= gyro_roll_cal;
    gyro_pitch -= gyro_pitch_cal;
    gyro_yaw -= gyro_yaw_cal;
		acc_x-=acc_x_cal;
		acc_y-=acc_y_cal;
		
		
    gyro_roll_input = (gyro_roll_input * 0.8) + (((float)gyro_roll / 65.5) * 0.2);
    gyro_pitch_input = (gyro_pitch_input * 0.8) + (((float)gyro_pitch / 65.5) * 0.2);
    gyro_yaw_input = (gyro_yaw_input * 0.8) + (((float)gyro_yaw / 65.5) * 0.2);
    
		angle_pitch += (float)gyro_pitch * 0.0000611;
    angle_roll += (float)gyro_roll * 0.0000611;
    angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);
    angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);

    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

    if (fabs(acc_x) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_x/acc_total_vector)* RAD_TO_DEG;
  }
  if (fabs(acc_y) < acc_total_vector) { 
    angle_roll_acc = asin((float)acc_y/acc_total_vector)* RAD_TO_DEG;
  }

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

    pitch_level_adjust = angle_pitch * 15;
    roll_level_adjust = angle_roll * 15;
		// Khóa setpoint ? v? trí hi?n t?i khi v?a b?t (CH3 nh?, CH4 l?n)
		if (!setpoint_locked && CH3 < 1050) {
				roll_neutral = CH1;
				pitch_neutral = CH2;
				yaw_neutral = CH4; // N?u c?n cho yaw
				setpoint_locked = 1;
		}
		if(CH3 > 1050)
				setpoint_locked = 0;


    pid_roll_setpoint_base = CH1;
    pid_pitch_setpoint_base = CH2;


    if (pid_roll_setpoint_base > 2000)pid_roll_setpoint_base = 2000;
		if (pid_roll_setpoint_base < 1000)pid_roll_setpoint_base = 1000;
		if (pid_pitch_setpoint_base > 2000)pid_pitch_setpoint_base = 2000;
		if (pid_pitch_setpoint_base < 1000)pid_pitch_setpoint_base = 1000;
    
		start_stop_takeoff();
		
		pid_roll_setpoint = 0;
		
		if(pid_roll_setpoint_base > roll_neutral)pid_roll_setpoint = pid_roll_setpoint_base - roll_neutral;
		else if(pid_roll_setpoint_base < roll_neutral)pid_roll_setpoint = pid_roll_setpoint_base - roll_neutral;

		pid_roll_setpoint -= roll_level_adjust;
		pid_roll_setpoint /= 4.0;

		pid_pitch_setpoint = 0;

		if(pid_pitch_setpoint_base > pitch_neutral)pid_pitch_setpoint = pid_pitch_setpoint_base - pitch_neutral;
		else if(pid_pitch_setpoint_base < pitch_neutral)pid_pitch_setpoint = pid_pitch_setpoint_base - pitch_neutral;

		pid_pitch_setpoint -= pitch_level_adjust;
		pid_pitch_setpoint /= 4.0;

		pid_yaw_setpoint = 0;

		if(CH3 > 1050){
			if(CH4 > yaw_neutral)pid_yaw_setpoint = (CH4 - yaw_neutral)/4.0;
			else if(CH4 < yaw_neutral)pid_yaw_setpoint = (CH4 - yaw_neutral)/4.0;
		}
		if (fabs(pid_roll_setpoint) < 10.0) pid_roll_setpoint = 0;
		if (fabs(pid_pitch_setpoint) < 10.0) pid_pitch_setpoint = 0;
		if (fabs(pid_yaw_setpoint) < 10.0) pid_yaw_setpoint = 0;

		calculate_pid();
		throttle = CH3;
		if (start == 2) {
        if (throttle > 1800) throttle = 1800; 

        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

        /*if (battery_voltage < 1240 && battery_voltage > 800) {
            float factor = (1240 - battery_voltage) / 3500.0;
            esc_1 += esc_1 * factor;
            esc_2 += esc_2 * factor;
            esc_3 += esc_3 * factor;
            esc_4 += esc_4 * factor;
        }*/

        if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;
				if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;
				if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;
				if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;

				if(esc_1 > 2000)esc_1 = 2000;
				if(esc_2 > 2000)esc_2 = 2000;
				if(esc_3 > 2000)esc_3 = 2000;
				if(esc_4 > 2000)esc_4 = 2000;
    } else {
        esc_1 = esc_2 = esc_3 = esc_4 = 1000;
    }

		Set_ESC_Throttle();
		while (micros() - current_time < 4000);
		current_time = micros();

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void calculate_pid(){
  // Tính toán Roll
  pid_error_temp_roll = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp_roll;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp_roll + pid_i_mem_roll + pid_d_gain_roll * ((pid_error_temp_roll - pid_last_roll_d_error));
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp_roll;

  // Tính toán Pitch
  pid_error_temp_pitch = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp_pitch;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp_pitch + pid_i_mem_pitch + pid_d_gain_pitch * ((pid_error_temp_pitch - pid_last_pitch_d_error));
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp_pitch;

  // Tính toán Yaw
  pid_error_temp_yaw = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp_yaw;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp_yaw + pid_i_mem_yaw + pid_d_gain_yaw * ((pid_error_temp_yaw - pid_last_yaw_d_error));
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp_yaw;
}
void gyro_signalen()
{
    uint8_t data[14];  // B? d?m luu d? li?u t? MPU6050

    // Ð?c 14 byte t? d?a ch? 0x3B c?a MPU6050
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS << 1, 0x3B, I2C_MEMADD_SIZE_8BIT, data, 14, HAL_MAX_DELAY);
		
    // Chuy?n d?i d? li?u t? byte sang s? nguyên 16-bit
    acc_x = (int16_t)(data[0] << 8 | data[1]);
    acc_y = (int16_t)(data[2] << 8 | data[3]);
    acc_z = (int16_t)(data[4] << 8 | data[5]);
    temperature = (int16_t)(data[6] << 8 | data[7]);
    gyro_roll = (int16_t)(data[8] << 8 | data[9]);
    gyro_pitch = (int16_t)(data[10] << 8 | data[11]);
    gyro_yaw = (int16_t)(data[12] << 8 | data[13]);

    // Ð?o chi?u gyro_pitch và gyro_yaw nhu trong mã g?c
    gyro_pitch *= -1;
    gyro_yaw *= -1;
    
}
void calibrate_gyro(void) {
    for (cal_int = 0; cal_int < 2000; cal_int++) {
        if (cal_int % 25 == 0) {
            GPIOC->ODR ^= GPIO_PIN_14;  // Ð?o tr?ng thái chân PC14
        }
        gyro_signalen();  // Ð?c giá tr? t? MPU6050
        gyro_roll_cal =gyro_roll_cal + gyro_roll;
        gyro_pitch_cal =gyro_pitch_cal + gyro_pitch;
        gyro_yaw_cal =gyro_yaw_cal + gyro_yaw;
				acc_x_cal =acc_x_cal + acc_x;
				acc_y_cal =acc_y_cal + acc_y;
				HAL_Delay(4);
    }
    GPIOC->BSRR = GPIO_PIN_14; // red_led(HIGH);
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;
		acc_x_cal/=2000;
		acc_y_cal/=2000;
		
}
void gyro_setup() {
    uint8_t data[2];

    // C?u hình Thanh ghi PWR_MGMT_1 (0x6B) - B?t MPU6050
    data[0] = 0x6B;
    data[1] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

    // C?u hình Thanh ghi GYRO_CONFIG (0x1B) - 500dps full scale
    data[0] = 0x1B;
    data[1] = 0x08;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);

    // C?u hình Thanh ghi ACCEL_CONFIG (0x1C) - ±8g full scale range
    data[0] = 0x1C;
    data[1] = 0x10;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
		
		data[0] = 0x1A;
    data[1] = 0x03;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 2, HAL_MAX_DELAY);
    // Ð?c l?i GYRO_CONFIG d? ki?m tra
    uint8_t check;
    data[0] = 0x1B;
    HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS << 1, data, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, MPU6050_ADDRESS << 1, &check, 1, HAL_MAX_DELAY);
    
    if (check != 0x08) {
        GPIOC->BSRR = GPIO_PIN_14; 
        while (1) HAL_Delay(10);
    }
}
void start_stop_takeoff(void) {
  if (CH3 < 1050 && CH4 < 1050)start = 1;
  // Kh?i d?ng d?ng co: c?n ga ? v? trí th?p và bên trái.
  if (start == 1 && CH3 < 1050 && CH4 > 1450) {
    throttle = motor_idle_speed;
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    acc_total_vector_at_start = acc_total_vector;
    start = 2;
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {
      takeoff_throttle = manual_takeoff_throttle - 1500;
      takeoff_detected = 1;
      // Ð?t l?i di?u khi?n PID d? c?t cánh suôn s?.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
    }
    else if (manual_takeoff_throttle) {
      error = 4;
      takeoff_throttle = 0;
      start = 0;
    }
  }
  // D?ng d?ng co: c?n ga ? v? trí th?p và bên ph?i.
  if (start == 2 && CH3 < 1050 && CH4 > 1950) {
    start = 0;
    takeoff_detected = 0;
  }

  if (takeoff_detected == 0 && start == 2) {
    if (CH3 > 1480 && throttle < 1750) throttle++;
    if (throttle == 1750)error = 5;
    if (CH3 <= 1480) {
      if (throttle > motor_idle_speed)throttle--;
      // Ð?t l?i di?u khi?n PID d? c?t cánh suôn s?.
      else {
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;
      else {
        error = 6;
        takeoff_throttle = 0;
      }
    }
  }

void error_signal(void) {
    if (error >= 100) {
        // Khi l?i >= 100, dèn luôn b?t
        GPIOC->BSRR = (1 << 14);  // SET PC14 lên m?c cao

    } else if (HAL_GetTick() > error_timer) {
        // N?u dã h?t th?i gian ch? gi?a các l?n nh?p nháy
        error_timer = HAL_GetTick() + 250; // Ð?t th?i gian ch? ti?p theo là 250ms

        if (error > 0 && error_counter > error + 3) {
            error_counter = 0; // Ð?t l?i b? d?m n?u vu?t quá s? l?n nh?p nháy c?n thi?t
        }

        if (error_counter < error && error_led == 0 && error > 0) {
            // Pha b?t LED
            GPIOC->BSRR = (1 << 14);  // SET PC14 lên m?c cao
            error_led = 1; // Ghi nh?n LED dang b?t
        } else {
            // Pha t?t LED
            GPIOC->BSRR = (1 << (14 + 16));  // RESET
            error_counter++; // Tang s? l?n nh?p nháy dã th?c hi?n
            error_led = 0;   // Ghi nh?n LED dang t?t
        }
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
