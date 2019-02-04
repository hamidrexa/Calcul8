/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include<math.h>
/* USER CODE BEGIN Includes */
#define mu1 mux5_Pin
#define mu2 mux6_Pin

#define a 7
#define b 6
#define c 5
#define d 4
#define e 3
#define f 2
#define g 1
#define dot 0
static int point=0;
static int segment=0;
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int getHexVal(char charachter)
{
  if(charachter >= '0' && charachter<= '9')
    return charachter - '0';
  else if(charachter >= 'a' && charachter<= 'f')
    return charachter - 'a' + 10;
  else if(charachter >= 'A' && charachter<= 'F')
    return charachter - 'A' + 10;
  else
    return -1;//error
}

int makepm_pax_rs232_a80(char* value,char*pm,int sizepm){
	int totalsize,sizepacket1,sizepacket2;
	char part1[250];
		totalsize=112+sizepm; 
		sizepacket1=106+sizepm;
		sizepacket2=23+sizepm;
		//sprintf(pm,"027400000001726E B11B 81");
		sprintf(pm,"02");
		sprintf(pm +2 ,"%x0000000172",totalsize);
		sprintf(pm+14,"%xB1",sizepacket1);
		sprintf(pm+18 ,"%x81",sizepacket2);
		sprintf(pm+22 ,"%.2x",sizepm);
		for ( int s=0;s<sizepm;s++){
			sprintf(pm+24+s*2 ,"3%c",value[s]);
		}
		sprintf(pm+24	+(sizepm*2),"820AD4E4C7D3E53A3130300A830101840103850100B24FA11B810431303030820130830130840130850130860130870130880130A230A10C8105D4E4C7D3E58203313030A1118106446C6C5665728207322E312E302E31A10D81065072675665728203312E3031C1E704");
		return strlen(pm);
//							for(int i = 0; i < 100; i +=2)
//							{
//								part1[i/2] = getHexVal(pm[i])*16 + getHexVal(pm[i+1]);
//								HAL_UART_Transmit(&huart1, (uint8_t*)(part1+i/2), 1, 5000);
//							}
}	
void mux_control_mode(){
	address0_GPIO_Port->ODR=0x2;
	address1_GPIO_Port->ODR=0x2;
	address2_GPIO_Port->ODR=0;
	
}
void beep (int time){
	for(int i=0;i<time;i++){
	buzzer_GPIO_Port->ODR^=buzzer_Pin;
		HAL_Delay(1);
	}
}

void delay (int i){
	

	while(i>0){
	TIM16->SR &=0xfe;
	  TIM16->CNT=0;
						while((!((TIM16->SR)&0x01)) );
			 
					 TIM16->CNT=0;
	i--;
	}
}
void wating(void){//watting untill common driver pin become zero
mux_control_mode();
		delay(5);
	while(1){
		 if( (mux3_GPIO_Port->IDR)&(mux3_Pin)){while((mux3_GPIO_Port->IDR)&(mux3_Pin));}
		 else{while(!((mux3_GPIO_Port->IDR)&(mux3_Pin)));break;}
		 
	 }
}
int duty_finder(){
	int duty=0;
	wating();
	while((mux3_GPIO_Port->IDR)&(mux3_Pin)){
		delay(1);
		duty++;
	 }
	return duty;
}
double rialtoman_checking(int duty){
	int input;
	mux_control_mode();
	int rial=0,toman=0;
	for (int i=0;i<2*duty;i++){
		delay(1);
		input=GPIOA->IDR;
		if(((input&mux1_Pin)<<3)==(input&mux4_Pin)){
			rial++;
		}
		if(((input&mux2_Pin)<<2)==(input&mux4_Pin)){
			toman++;
		}
	
}
	if((rial<600)&(toman<600))return 1.0;
	if(rial>toman){
		return 10000.0;
	}
	else if( rial<toman)return 10.0;
	else return 1.0;
	}
	

/* USER CODE END 0 */
double find_number(int *signal_0,int *signal_1,int *signal_2,int dA,int dB,int dC,int Amux,int Bmux,int Cmux ){
			double x1;
			int hex=0;
			hex|=((signal_0[dA]&Amux)&&Amux)<<b;//d3--mux1 ->num 1 b column 1 
			hex|=((signal_0[dB]&Bmux)&&Bmux)<<a;//d2--mux1 ->num 1 a column 2 
			hex|=((signal_0[dC]&Cmux)&&Cmux)<<f;//d2--mux2 ->num 1 f	column 3 
			hex|=((signal_1[dA]&Amux)&&Amux)<<c;//d3--mux1 ->num 1 c column 1 
			hex|=((signal_1[dB]&Bmux)&&Bmux)<<g;//d2--mux1 ->num 1 g column 2 
			hex|=((signal_1[dC]&Cmux)&&Cmux)<<e;//d2--mux2 ->num 1 e	column 3 
			hex|=((signal_2[dA]&Amux)&&Amux)<<dot;//d3--mux1 ->num  dot column 1 
			hex|=((signal_2[dB]&Bmux)&&Bmux)<<d;//d2--mux1 ->num 1 d column 2 
	
	switch (hex|0x01){
			case 0x03:{x1=0;break;}
			case 0x9f:{x1=1;break;}
			case 0x25:{x1=2;break;}
			case 0x0d:{x1=3;break;}
			case 0x99:{x1=4;break;}
			case 0x49:{x1=5;break;}
			case 0x41:{x1=6;break;}
			case 0x1b:{x1=7;break;}
			case 0x01:{x1=8;break;}
			case 0x09:{x1=9;break;}
					 default:{x1=0;break;}
			
		}
		if(hex&0x01){
			segment++;
		}
		else{
			point=segment;
		}
	return x1;
}
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();

  /* USER CODE BEGIN 2 */
	uint8_t muxinput[6];
	char pm_pos[300];
	//char pm_pos_hex[300];
	char print[300];
	char sum_print[30];
	int mu2_counter,mu1_counter,mu12_counter,mu_valid;
	int mu1_list[10],mu2_list[10];
	int signal_0[15],signal_1[15],signal_2[15];
	int numbers[15];
	int duty;
	int test,scale;
	double x1;
	
  /* USER CODE BEGIN 2 */
	HAL_IWDG_Start(&hiwdg);
	HAL_UART_Transmit(&huart1,(uint8_t*)"start\n",10,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	mux_control_mode();

	while (1)
		
 {
	HAL_IWDG_Refresh(&hiwdg);

	mu2_counter=0;
	mu1_counter=0;
	mu_valid=0;
	
	/*if(((GPIOA->IDR)&(mu2|mu1))== 0){//some keyys pressed (interupt)
			HAL_UART_Transmit(&huart1,(uint8_t*)"1\n",10,1);
	for(int i=0;i<8;i++){
		mu2_counter=0;
		mu1_counter=0;
				HAL_IWDG_Refresh(&hiwdg);
			while (((GPIOA->IDR)&(mu1|mu2))!= 0);//waiting until both 0
			while (((GPIOA->IDR)&(mu1|mu2))== 0){
				mu1_counter++;
			}
					HAL_IWDG_Refresh(&hiwdg);
				while (((GPIOA->IDR)&(mu1|mu2))!= (mu1|mu2));//waiting until both 1
				while (((GPIOA->IDR)&(mu1|mu2))== (mu1|mu2)){
					mu2_counter++;
				}
				mu1_list[i]=mu1_counter;
				mu2_list[i]=mu2_counter;
	}
}
		
		for(int i=0;i<8;i++){///validateion
			if(abs(mu1_list[i]-1760)<300){
				if(abs(mu2_list[i]-11640)<800){
					mu_valid++;
				}
			}
			mu1_list[i]=0;
			mu2_list[i]=0;
		}
	
	if(mu_valid>5){//mu_selection validate
	}
*/	//mu press mod 1 checking
//mu press mode 2 cheking
 mu1_counter=0;
 while(((GPIOA->IDR)&(mu2|mu1))== 0) mu1_counter++;
 if(mu1_counter>26000){//some key pressed
		mu1_counter=0;
	 while((((GPIOA->IDR)&(mu1))<<1)&&((GPIOA->IDR)&(mu2)));
	 if((((GPIOA->IDR)&(mu2|mu1))==0)){//mu detected
			HAL_IWDG_Refresh(&hiwdg);
			
			beep(100);
			scale=rialtoman_checking(duty_finder());
//		 sprintf(print,"scale:%d\n",scale);
//			HAL_UART_Transmit(&huart1,(uint8_t*)print,30,1);
			//HAL_Delay(1000);
		///start reading 
			for(int n=0;n<1;n++){
				duty=duty_finder();
				for(int i=0;i<15;i++){
							numbers[i]=0;
							signal_0[i]=0;
							signal_1[i]=0;
							signal_2[i]=0;
				}
				///reading signal_ parallel  state 0
				wating();
				address0_GPIO_Port->ODR=0; 
				address2_GPIO_Port->ODR=0;
				for (int di=0;di<4;di++){
					delay(5);
					signal_0[di]=(GPIOA->IDR);
			  	address0_GPIO_Port->ODR+=0x1; 
					
			  }
				address0_GPIO_Port->ODR=0; 
				address2_GPIO_Port->ODR=0xff;
				for (int di=4;di<8;di++){
					delay(5);
					signal_0[di]=(GPIOA->IDR);
			  	address0_GPIO_Port->ODR+=0x1; 
					
			}//reading finished  state 0
				wating();
				delay(duty/3+5);
			
				///reading signal_ parallel state 1
				address0_GPIO_Port->ODR=0; 
				address2_GPIO_Port->ODR=0;
				for (int di=0;di<4;di++){
				  delay(5);
		   		signal_1[di]=(GPIOA->IDR);
			   	address0_GPIO_Port->ODR+=0x1;
		 
			}
				address0_GPIO_Port->ODR=0; 
				address2_GPIO_Port->ODR=0xff;
				for (int di=4;di<8;di++){
					delay(5);
					signal_1[di]=(GPIOA->IDR);
			  	address0_GPIO_Port->ODR+=0x1; 
				}//reading finished state 1
				wating();
				delay(duty*4/5);
			
				///reading signal_ parallel state 2
				address0_GPIO_Port->ODR=0; 
				address2_GPIO_Port->ODR=0;
				for (int di=0;di<4;di++){
					delay(5);
					signal_2[di]=(GPIOA->IDR);
			  	address0_GPIO_Port->ODR+=0x1; 
				}
				address0_GPIO_Port->ODR=0; 
				address2_GPIO_Port->ODR=0xff;
				for (int di=4;di<8;di++){
				delay(5);	
					signal_2[di]=(GPIOA->IDR);
			  	address0_GPIO_Port->ODR+=0x1; 
			}//reading finished
	
				x1=0;
				segment=0;
				x1=find_number(signal_0,signal_1,signal_2,3,2,2,mux1_Pin,mux1_Pin,mux2_Pin);
				x1+=pow(10,1)*find_number(signal_0,signal_1,signal_2,1,0,7,mux1_Pin,mux1_Pin,mux1_Pin);
				x1+=pow(10,2)*find_number(signal_0,signal_1,signal_2,4,5,6,mux1_Pin,mux1_Pin,mux1_Pin);
				x1+=pow(10,3)*find_number(signal_0,signal_1,signal_2,7,6,5,mux2_Pin,mux2_Pin,mux2_Pin);
				x1+=pow(10,4)*find_number(signal_0,signal_1,signal_2,4,0,1,mux2_Pin,mux2_Pin,mux2_Pin);
				x1+=pow(10,5)*find_number(signal_0,signal_1,signal_2,3,7,0,mux2_Pin,mux3_Pin,mux3_Pin);
				x1+=pow(10,6)*find_number(signal_0,signal_1,signal_2,1,6,3,mux3_Pin,mux3_Pin,mux3_Pin);
				x1+=pow(10,7)*find_number(signal_0,signal_1,signal_2,5,4,7,mux3_Pin,mux3_Pin,mux4_Pin);
				x1+=pow(10,8)*find_number(signal_0,signal_1,signal_2,6,3,5,mux4_Pin,mux4_Pin,mux4_Pin);
				x1+=pow(10,9)*find_number(signal_0,signal_1,signal_2,4,7,6,mux4_Pin,mux5_Pin,mux5_Pin);
				x1+=pow(10,10)*find_number(signal_0,signal_1,signal_2,5,4,0,mux5_Pin,mux5_Pin,mux5_Pin);
				x1+=pow(10,11)*find_number(signal_0,signal_1,signal_2,1,3,0,mux5_Pin,mux5_Pin,mux6_Pin);
				x1+=pow(10,12)*find_number(signal_0,signal_1,signal_2,4,5,6,mux6_Pin,mux6_Pin,mux6_Pin);
				x1+=pow(10,13)*find_number(signal_0,signal_1,signal_2,7,3,1,mux6_Pin,mux6_Pin,mux6_Pin);
				x1=x1/pow(10,point);
				x1=x1*scale;
			for( int i=0;i<30;i++){
								sum_print[i]=0;
								
								}		
			 	sprintf(sum_print,"%.14f\n",x1);
				//HAL_UART_Transmit(&huart1,(uint8_t*)sum_print,30,1);
				 for(int i=0;i<300;i++){
								
								pm_pos [i]=0;
								}			
				for( int i=0;i<30;i++){
								sum_print[i]=0;
								
								}							 
				sprintf(sum_print,"%0.f",x1);/// rounding
				//HAL_UART_Transmit(&huart1,(uint8_t*)sum_print,30,1);
				int len =strlen(sum_print);
				makepm_pax_rs232_a80(sum_print, pm_pos, len);
				//HAL_UART_Transmit(&huart1,(uint8_t*)pm_pos,300,1);
				uint8_t bit=0;
				for(int i = 0; i < (240+len); i +=2)
							{
								bit=getHexVal(pm_pos[i])*16 + getHexVal(pm_pos[i+1]);
								HAL_UART_Transmit(&huart1,&bit,1,5000);//send vaue pm in protocol
							}
				mux_control_mode();
		}
			
	
	 }

}
  }
 

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
	TIM16->PSC |=1; /* (1) */
TIM16->ARR = 500- 1; /* (2) */
TIM16->CCR1 = 500- 1; /* (3) */  
TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
TIM16->CCER |= TIM_CCER_CC1E; /* (5)*/
TIM16->BDTR |= TIM_BDTR_MOE; /* (6) */
TIM16->CR1 |= TIM_CR1_CEN; /* (7) */

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, address0_Pin|address1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(buzzer_GPIO_Port, buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(address2_GPIO_Port, address2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : address0_Pin address1_Pin */
  GPIO_InitStruct.Pin = address0_Pin|address1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : buzzer_Pin */
  GPIO_InitStruct.Pin = buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : mux1_Pin mux2_Pin mux3_Pin mux4_Pin 
                           mux5_Pin mux6_Pin */
  GPIO_InitStruct.Pin = mux1_Pin|mux2_Pin|mux3_Pin|mux4_Pin 
                          |mux5_Pin|mux6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : address2_Pin */
  GPIO_InitStruct.Pin = address2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(address2_GPIO_Port, &GPIO_InitStruct);

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
