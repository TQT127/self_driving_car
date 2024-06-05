/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ps2.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
float Key[4]={0};

volatile float rate = 0;
volatile float averageDistance=0;

uint8_t txBuffer[8];
char mode;
volatile int steer = 3000;
volatile int speed = 0;
int PS2_KEY;

volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t Difference = 0;
volatile uint8_t Is_First_Captured = 0;  // is the first value captured ?
float Distance_Front  = 0.0;

#define r 3.2 //ban kinh banh xe (cm)
#define pi 3.14159 
volatile int32_t encoder_cnt = 0;	
volatile int32_t pre_encoder_cnt = 0;	
volatile int32_t cnt_pulse = 0;
volatile float current_speed = 0;
volatile float speed_car = 0; //cm/s

#define NUM_READINGS 20
float distanceReadings[NUM_READINGS];
int readIndex = 0;
float totalDistance = 0;

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/*--------------------------------------------------*/
void delay (uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER (&htim2) < time);
}
/*--------------------------------------------------*/
void Enable_motor_forward()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
}
void Enable_motor_backward()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);
}
void Stop_motor()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	htim4.Instance->CCR4 = 3000;
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
}
void Run_Car()
{
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
}
void Stop_Car()
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
	htim4.Instance->CCR4 = 3000;
	htim2.Instance->CCR2 = 0;
}
/*--------------------------------------------------*/
volatile uint32_t prev_capture = 0;
volatile uint32_t delta_ticks = 0;
volatile float speed_setpoint = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{	
	if (htim->Instance == TIM1)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			if (Is_First_Captured==0) // if the first value is not captured
			{
				IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
				Is_First_Captured = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}
			else if (Is_First_Captured==1)   // if the first is already captured
			{
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
				if (IC_Val2 > IC_Val1)
				{
					Difference = IC_Val2-IC_Val1;
				}
				else if (IC_Val1 > IC_Val2)
				{
					Difference = (0xffff - IC_Val1) + IC_Val2;
				}
				Distance_Front = Difference * .034/2;
				if(Distance_Front<2.0) Distance_Front=2.0;
				else if(Distance_Front>450.0) Distance_Front=450.0;
				Is_First_Captured = 0; // set it back to false
				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			}
		}
	}
}
/*--------------------------------------------------*/
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
void HCSR04_Read_Fr (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
/*--------------------------------------------------*/
float scale_range(float in_value, float in_low, float in_high, float out_low, float out_high ){
	float in_range = in_high - in_low;
	float out_range = out_high - out_low;
	float out_value = ((in_value - in_low) * out_range / in_range) + out_low;
	return out_value;
}
/*--------------------------------------------------*/
#define RX_SIZE 8
static uint8_t rx_buffer[RX_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
// Determine which UART triggered the callback
  if (huart == &huart2) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    // Handle huart2 data
    if (rx_buffer[0] == 'g') {
      //sprintf((char*)(txBuffer), "%d,%d\r\n", steer, speed);
      //HAL_UART_Transmit_DMA(&huart2, txBuffer, sizeof(txBuffer));
    }
		else if(rx_buffer[0] == 'E') Enable_motor_forward();
		else if(rx_buffer[0] == 'S') Stop_motor();
		else 
		{
			steer = (rx_buffer[0]-48)*1000+(rx_buffer[1]-48)*100+(rx_buffer[2]-48)*10+(rx_buffer[3]-48);
			speed_setpoint = (rx_buffer[4]-48)*10+(rx_buffer[5]-48);
    }
    //memset(rx_buffer, 0, 8);
  }
}
/*--------------------------------------------------*/
int steer_out = 3000;
float set_scale_steer (float steer_in){
	if(steer_in == 128) steer_out = 3000;
	else{
	steer_out = scale_range(steer_in,0,255,1800,4200);
	}
	htim4.Instance->CCR4 = steer_out;
	return steer_out;
}
/*--------------------------------------------------*/
int speed_out = 0;
int set_scale_speed (float speed_in){
	if(speed_in >= 0 && speed_in < 128){
			Enable_motor_forward();
			speed_out = scale_range(speed_in,128,0,0,1000);
	}
	else if (speed_in > 128 && speed_in <= 255){
			Enable_motor_backward();
			speed_out = scale_range(speed_in,128,255,0,1000);	
	}
	else if (speed_in == 128) speed_out = 0; 
	htim2.Instance->CCR2 = speed_out;
	return speed_out;
}
/*--------------------------------------------------*/
float filteredDistance(float newDistance){
  totalDistance = totalDistance - distanceReadings[readIndex] + newDistance;
  distanceReadings[readIndex] = newDistance;
  readIndex = (readIndex + 1) % NUM_READINGS;
  // Calculate and return the average
  return totalDistance / NUM_READINGS;
}

/* --------------------------PID------------------------------- */


#define SAMPLE_TIME 10 // thoi gian lay mau (ms)
float ek = 0.0; // sai so trong PID
float ek1 = 0.0; // sai so lan lap truoc
float ek2 = 0.0; // sai so lan lap truoc nua
float uk=0.0,uk1=0.0;

float pid_kp = 1.0;   // PID coefficient (Proportional)
float pid_ki = 0.009; // PID coefficient (Integral)
float pid_kd = 0.0;   // PID coefficient (Derivative)
float pid_update(float speed_measurement)
{
	ek = speed_setpoint - speed_measurement;
	uk = uk1 + pid_kp*(ek-ek1) + pid_ki*SAMPLE_TIME*0.5*(ek+ek1) + pid_kd*(ek-2*ek1+ek2)/SAMPLE_TIME; 
	if(uk < 0) uk = 0;
	else if(uk > 100) uk = 100;
	ek2=ek1;
	ek1=ek;
	uk1=uk;
	return uk;
}
/*--------------------------------------------------*/
void Xuat_PWM(TIM_HandleTypeDef *htim, uint32_t Channel, float Duty_Cycle){
Duty_Cycle = Duty_Cycle / 100 * htim->Instance->ARR;
__HAL_TIM_SET_COMPARE(htim, Channel, (uint16_t)Duty_Cycle);
}
/*--------------------------------------------------*/
int cnt_1ms = 0;
int cnt_10ms = 0;
int duty_cycle = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) { //ngat moi 1ms
	cnt_1ms ++;
	if(cnt_1ms==10) //ngat moi 10ms
	{ 
		cnt_1ms = 0;
		cnt_10ms ++;
		encoder_cnt = __HAL_TIM_GET_COUNTER(&htim3);
		cnt_pulse = abs(encoder_cnt - pre_encoder_cnt); //so xung/5ms
		if(cnt_pulse > 0x7fff){
			cnt_pulse = abs(0xffff - cnt_pulse);
		}
		pre_encoder_cnt = encoder_cnt;
		current_speed = ((float)cnt_pulse*100.0f/(234.3f*4.0f))*60.0f;
		speed_car = (int)cnt_pulse*100.0f/(234.3f*4.0f*2.7f)*(2.0f*pi*r); //cm/s
		duty_cycle = (int)(pid_update(speed_car));
		HCSR04_Read_Fr();
		averageDistance = filteredDistance(Distance_Front);
		if(cnt_10ms==10){
			cnt_10ms = 0;
		sprintf((char*)txBuffer, "%03d,%02d\r\n",(int)averageDistance,(int)speed_car);
    HAL_UART_Transmit_DMA(&huart2, txBuffer, sizeof(txBuffer));
		}
		if(averageDistance < 20.0)
		{
			htim2.Instance->CCR2 = 0;
		}
		else
		{
		htim4.Instance->CCR4 = steer;
		Xuat_PWM(&htim2,TIM_CHANNEL_2,duty_cycle);					
		}
	}
}
}
/*--------------------------------------------------*/
void init_main(){
	htim2.Instance->CCR2 = 0;
  htim4.Instance->CCR4 = 3000;
	PS2_SetInit();
	HAL_Delay(100);
}

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

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);//sr04
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, RX_SIZE);
__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	HAL_UART_Transmit_DMA(&huart2, txBuffer, sizeof(txBuffer));
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2); // Encoder
	HAL_TIM_Base_Start_IT(&htim2);
	init_main();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
		{	
			//HCSR04_Read_Fr();
    /* USER CODE END WHILE */
		Key[2] = PS2_RedLight();
		PS2_ClearData();
		HAL_Delay(1);
		PS2_ReadData();
		Key[0] = PS2_AnalogData(PSS_LY);
		Key[1] = PS2_AnalogData(PSS_RX);
		HAL_Delay(1);
		if(Data[1]==115){
		speed = set_scale_speed(Key[0]);
		steer = set_scale_steer(Key[1]);
		}
	}
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, CS_Pin|CLK_Pin|CMD_Pin|GPIO_PIN_4
                          |GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_Pin CLK_Pin CMD_Pin */
  GPIO_InitStruct.Pin = CS_Pin|CLK_Pin|CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DAT_Pin */
  GPIO_InitStruct.Pin = DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_USART1_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
