/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// security's variables
int isLock = 0; // check system is protected
int isHack = 0; // check system is hacked
int unLock = 0; // check unlocking
int togPin = 0; // toggle LED when unlock the system
//-----------------------------------

// Audio's variables
uint8_t initV[2];
uint16_t Istr[100];
uint8_t note[] = {0x92,0xD2,0x82,0xA2,0xC2};
//-----------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
// Interrupt function for push button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	isLock=1;
	togPin=0;
}
//-----------------------------------

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// Microphone's function and variables
float float_abs(float in){
	return in < 0 ? -in : in;
}

#define PDM_BUFFER_SIZE 20
#define PCM_BUFFER_SIZE 20
#define LEAKY_KEEP_RATE 0.95
#define UART_DEBUG_TICK_RATE 100
#define PDM_BLOCK_SIZE_BITS 16
//-----------------------------------

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	// Microphone's function and variables
	uint8_t i;
  	uint16_t pdm_buffer[PDM_BUFFER_SIZE]; // Buffer for pdm value from hi2s2 (Mic)
	uint16_t pdm_value = 0;				// For keeping pcm value calculated from pdm_value
	uint8_t pcm_value = 0;				// value range is 0-16, 8-bit is chosen because it can store 0-255

	char uart_temp_display_buffer[100];

	float leaky_pcm_buffer = 0.0;			// Fast Estimation of moving average of PDM
	float leaky_amp_buffer = 0.0; 		// Fast Estimation of moving average of abs(PCM)
	float max_amp = 0;
	//-----------------------------------


  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  InitSound(); // Init Audio
  /* USER CODE BEGIN 2 */

  // accelerometer's variables
  uint8_t addr;
  uint8_t data;
  uint8_t x,y,z = 0;
  //-----------------------------------

  // Initialize accelerometer
  //set CS = 0 start SPI
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);

  addr = 0x20; // control register
  HAL_SPI_Transmit(&hspi1,&addr,1,50);

  data = 0x47; // active X Y Z and full scale is +-2.0g
  HAL_SPI_Transmit(&hspi1,&data,1,50);

  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
  //-----------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //start accelerometer
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

	  // OUT_X
	  addr = 0x29 | 0x80;
	  HAL_SPI_Transmit(&hspi1,&addr,1,50);
	  HAL_SPI_Receive(&hspi1,&x,1,50);

	  // OUT_Y
	  addr = 0x2B | 0x80;
	  HAL_SPI_Transmit(&hspi1,&addr,1,50);
	  HAL_SPI_Receive(&hspi1,&y,1,50);

	  // OUT_Z
	  addr = 0x2D | 0x80;
	  HAL_SPI_Transmit(&hspi1,&addr,1,50);
	  HAL_SPI_Receive(&hspi1,&z,1,50);
	  //-----------------------------------


	  //-127 to 127
	  if(isHack==1){ // if system is hacked
		  // LED and sound alert
		  WriteAllLed(GPIO_PIN_SET);
		  PlaySound(0);
		  HAL_Delay(10);
		  WriteAllLed(GPIO_PIN_RESET);
		  PlaySound(1);
		  HAL_Delay(100);
	  }
  	  else{ // if system is not hacked
  		  if(isLock==1){ // if system is protected

  			  // Initialize Microphone for unlocking.
  			  HAL_I2S_Receive(&hi2s2,pdm_buffer,PDM_BUFFER_SIZE,1000); // Receive PDM from Mic

  			  // Unlocking
		  	  if(unLock >=4) {
		  		  isLock = 0;
		  		  unLock = 0;
		  		  HAL_Delay(100);
		  		  PlayUnlock();
		  	  }

		  	  //Calculating voice
		  	  for(i=0;i<PDM_BUFFER_SIZE;i++){
		  		  pcm_value = 0;
		 		  pdm_value = pdm_buffer[i];
		 		  //calculate PCM value
		 		  while(pdm_value!=0){
		 			  pcm_value++;
		 			  pdm_value ^= pdm_value & -pdm_value;
		 		  }
		 		  leaky_pcm_buffer += pcm_value;
		 		  leaky_pcm_buffer *= LEAKY_KEEP_RATE;
		 		  leaky_amp_buffer += float_abs(leaky_pcm_buffer);
		 		  leaky_amp_buffer *= LEAKY_KEEP_RATE;
		 	  }
		 	  max_amp = leaky_amp_buffer;
		 	  if(max_amp > 3000){
		 		  unLock++;
		 	  }
		 	  else{
		 		 if(unLock-1<0) unLock = 0;
		 		 else unLock--;
		 	  }
		 	  max_amp = 0;
		 	  HAL_Delay(90);
		 	  HAL_GPIO_TogglePin(GPIOD, 1<<(12+togPin));
		 	  togPin=(togPin+1)%4;

		 	  // if board moves, that means system is hacked.
		 	  if (z <= -30 || z>= 85 || (y>10&&y<60) || (x<246&&x>200) || (x>10&&x<60) || (y<246&&y>200)){
		 		  isHack=1;
		 		  WriteAllLed(GPIO_PIN_SET);
		 		  HAL_Delay(100);
		 		  WriteAllLed(GPIO_PIN_RESET);
		 		  HAL_Delay(10);
		 	  }
  		  }
  		  else{ // if system is not protected
  			  if (z <= -30 || z>= 85) {
  				  WriteAllLed(GPIO_PIN_SET);
  				  HAL_Delay(500);
  				  WriteAllLed(GPIO_PIN_RESET);
  				  HAL_Delay(10);}

  			  // you can move the board
  			  else{
  				  if(y>10&&y<60)HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
  				  else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
  				  //left green
  				  if(x<246&&x>200)HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
  				  else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
  				  //right red
  				  if(x>10&&x<60)HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
  				  else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
  				  //down blue
  				  if(y<246&&y>200)HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
  				  else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
  			  }
  		  }
  	  }
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

	  // UART status of the system
	  if(isLock==1) {
		  HAL_UART_Transmit(&huart2,(uint8_t*)"\rSystem Status : Locked", 23, 100000);
	  }
	  else{
		  HAL_UART_Transmit(&huart2,(uint8_t*)"\rSystem Status : unLock", 23, 100000);
	  }
	  if(isHack==1) {
		  HAL_UART_Transmit(&huart2,(uint8_t*)"\rSystem Status : Hacked", 23, 100000);
	  }
	  HAL_Delay(20);
  }
  /* USER CODE END 3 */

}

void WriteAllLed(GPIO_PinState PinState){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, PinState);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, PinState);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, PinState);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, PinState);
}
void PlayUnlock(){
	HAL_Delay(300);
	PlaySound(2);
	HAL_Delay(20);
	PlaySound(3);
	HAL_Delay(20);
	PlaySound(4);
}


void PlaySound(int r){

	if(r < 0 || r > 4) {
		HAL_Delay(25);
		return;
	}
	// Off
	initV[0] = 0x1E;
	initV[1] = 0x20;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);


	// Change note
	initV[0] = 0x1C;
	initV[1] = note[r];
	HAL_I2C_Master_Transmit (&hi2c1, 0x94, initV, 2, 50);

	// On
	initV[0] = 0x1E;
	initV[1] = 0xE0;
	HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50);

	int i;
	for(i=0;i<100;i++) HAL_I2S_Transmit (&hi2s3, Istr , 100, 10);
}

void InitSound(){

	  Istr[0] = 0;
	  //not sure if this is needed but i just put it here
	  if( HAL_I2S_Transmit (&hi2s3, Istr , 0x10, 10 ) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
	  }
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);  //Reset is set down
	  //confirmation LED
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	  //Initialization sequence for CS43L22:
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1); //Reset is set Up (Power CS43L22)
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);

	  initV[0] = 0x00;
	  initV[1] = 0x99;
	  //check if transfer failed. If so: turn on Red LED
	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	  }
	  initV[0] = 0x47;
	  initV[1] = 0x80;
	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	  }
	  initV[0] = 0x32;
	  initV[1] = 0x80; // 0xBB or 0x80
	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  }
	  initV[0] = 0x32;
	  initV[1] = 0x00; // 0x3B or 0x00
	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
	  }
	  initV[0] = 0x00;
	  initV[1] = 0x00;
	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	  }
	  initV[0] = 0x1E;
	  initV[1] = 0xC0;
	  if(HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50) != HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	  }
	  initV[0] = 0x02;
	  initV[1] = 0x9E;
	  if( HAL_I2C_Master_Transmit(&hi2c1, 0x94, initV, 2, 50)!= HAL_OK){
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 1);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
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

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1680;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
