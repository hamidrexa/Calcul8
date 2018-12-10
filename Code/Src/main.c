/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "string.h"
#include<stdio.h>
#include<math.h>
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
uint8_t aTxBuffer1[] = "RXNE wake-up successful";


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
//static void MX_TIM1_Init(void);
//static void MX_TIM3_Init(void);
//static void MX_TIM14_Init(void);
//static void MX_TIM17_Init(void);
int getHexVal(char c);
void send(char* hex);
void delay (int i){
	while(i>0){
	TIM16->SR &=0xfe;
	  TIM16->CNT=0;
						while((!((TIM16->SR)&0x01)) );
			 
					 TIM16->CNT=0;
	i--;
	}
}
void wating(void){
	//GPIOA->ODR&=0x80;
GPIOA->ODR=0x10; 
		delay(3);
	while(1){
		 if( (GPIOB->IDR)&(0x20)){while((GPIOB->IDR)&(0x20));}
		 else{while(!((GPIOB->IDR)&(0x20)));break;}
		 
	 }
}
  int main(void)
{
	HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

	
	
__HAL_RCC_USART1_CONFIG(RCC_USART1CLKSOURCE_HSI);
RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; /* (1) */
RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (2) */
RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /* (2) */
RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /* (1) */

//GPIOB->MODER = GPIO_MODER_MODER2_0;
//GPIOB->OSPEEDR=GPIO_OSPEEDR_OSPEEDR0_Msk;
	//GPIOB->PUPDR=GPIO_PUPDR_PUPDR0_1;
	
	GPIOA->MODER = GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_0|GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0;
 
	GPIOA->PUPDR=GPIO_PUPDR_PUPDR0_1|GPIO_PUPDR_PUPDR1_1 |GPIO_PUPDR_PUPDR4_1|GPIO_PUPDR_PUPDR5_1|GPIO_PUPDR_PUPDR6_1
	|GPIO_PUPDR_PUPDR7_1|GPIO_PUPDR_PUPDR8_1|GPIO_PUPDR_PUPDR9_1|GPIO_PUPDR_PUPDR11_1;
TIM16->PSC |=1; /* (1) */
TIM16->ARR = 500- 1; /* (2) */
TIM16->CCR1 = 500- 1; /* (3) */  
TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
TIM16->CCER |= TIM_CCER_CC1E; /* (5)*/
TIM16->BDTR |= TIM_BDTR_MOE; /* (6) */
TIM16->CR1 |= TIM_CR1_CEN; /* (7) */

TIM1->PSC |=8; /* (1) */
TIM1->ARR = 60000- 1; /* (2) */
TIM1->CCR1 = 60000- 1; /* (3) */
TIM1->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; /* (4) */
TIM1->CCER |= TIM_CCER_CC1E; /* (5)*/
TIM1->BDTR |= TIM_BDTR_MOE; /* (6) */
TIM1->CR1 |= TIM_CR1_CEN; /* (7) */



int i,j,s;
char buf8[100];
char buf5[50];
char buf[60];
char hex2[400];
char bufpax[250];
char arduino[14];
double   x1;
 
 
 
huart1.Instance = USART1;
HAL_UART_DeInit(&huart1); 
//huart1.Init.BaudRate   = 9600;//pax_ethernet
huart1.Init.BaudRate   = 115200;//pax_rs232
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits   = UART_STOPBITS_1;
huart1.Init.Parity     = UART_PARITY_NONE; 
huart1.Init.Mode       = UART_MODE_TX_RX; 
HAL_UART_Init(&huart1) ;


//sprintf(hex2,"54454A415241545F42414E4B30323030303030303030303430303030303030303030303130303030313220202020202020202020202020202020202020202020202000006EB1C5B923FAF77A");

NVIC_EnableIRQ(SysTick_IRQn); /* (1) */
NVIC_SetPriority(SysTick_IRQn,2); /* (2) */

TIM1->SR &=0xfe;
// while(1){

// 
// 
// }		
	int pa1,pa2,pa3,pb1,pb2,pb3;
	double sum;
	double sumatinon[3];
	double cont;
	double m,lastm;
	
	int pointer,duty;
		int*A;
	A = (int*) malloc (100);
	int*B;
	B = (int*) malloc (100);
  		GPIOA->ODR=0x10; 
			
 wating();

duty=0;
	while((GPIOB->IDR)&(0x20)){
		delay(1);
		duty++;
		 
	 }
 for( i=0;i<50;i++){
								buf5[i]=0;
								}
						 
							 sprintf(buf5,"%f",200);
							int len =strlen(buf5);							
							sprintf(hex2,"54454A415241545F42414E4B303230303030303030303034303%c3%c3%c3%c3%c3%c3%c3%c3%c3%c3%c",buf5[len-18],buf5[len-17],buf5[len-16],buf5[len-15],buf5[len-14], buf5[len-13],buf5[len-12],buf5[len-11],buf5[len-10],buf5[len-9],buf5[len-8] );
							char asciiStr[100];
							for(int i = 0; i < 151; i +=2)
							{
								asciiStr[i/2] = getHexVal(hex2[i])*16 + getHexVal(hex2[i+1]);
								HAL_UART_Transmit(&huart1, (uint8_t*)(asciiStr+i/2), 1, 5000);
							}
while (1)
  {
 
// cont++;
//		if(cont>900000){
//					sprintf(buf, "alive\n");
//  HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000); 
//			cont=0;
//			
//		}
//		GPIOA->ODR=0x10; 
//				wating();

//duty=0;
//	while((GPIOB->IDR)&(0x20)){
//		delay(1);
//		duty++;
//		 
//	 }
//			sprintf(buf,"D:%d\n",duty);
// 				HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);

//		
// ///send
// i=0;
// 
//		
//		pa1=0x30;
//pa2=0x20;
//pa3=0x10;
//		
//pb1=0x01;
//pb2=0x01;
//pb3=0x01;
//i=0;
//x1=num(pa1,pa2,pa3,pb1,pb2,pb3,&pointer,duty);

//			sprintf(buf,"D:%f\n",x1);
// 				HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
 i=0;
 while (((GPIOA->IDR)&(0x600))== 0){
	 

 i++;
 }

 if(i>26000){
	 
//	 	sprintf(buf,"D:%d\n",i);
// 			HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
	 while((((GPIOA->IDR)&(0x200))<<1)&&((GPIOA->IDR)&(0x400)));
	 if((((GPIOA->IDR)&(0x600))==0)){

		 for(s=0;s<3;s++)
		 {
			 GPIOA->ODR=0x10; 
			 			  wating();

duty=0;
	while((GPIOB->IDR)&(0x20)){
		delay(1);
		duty++;
		 
	 }
			 for(i=0;i<15;i++){
	A[i]=0;
	B[i]=0;
	
}

	
		 wating();
		 GPIOA->ODR=0x00; 
		 for (i=0;i<8;i++){
		delay(2);
			 B[i]=(GPIOB->IDR);
			  	GPIOA->ODR+=0x10; 
			 
		 }
		 A[0]|=(B[3]&0x01)<<6;
		 A[0]|=(B[2]&0x01)<<7;
		 A[0]|=(B[1]&0x01)<<2;
		 
		 A[1]|=(B[0]&0x01)<<6;
		 A[1]|=(B[4]&0x01)<<7;
		 A[1]|=(B[5]&0x01)<<2;
		
		 A[2]|=(B[6]&0x01)<<6;
		 A[2]|=(B[7]&0x01)<<7;
		 A[2]|=(B[4]&0x02)<<1;
		 
		  A[3]|=(B[2]&0x02)<<5;
		  A[3]|=(B[1]&0x02)<<6;
		  A[3]|=(B[0]&0x02)<<1;
		 
		 
		  A[4]|=(B[5]&0x02)<<5;
		  A[4]|=(B[6]&0x02)<<6;
		  A[4]|=(B[7]&0x02)<<1;
		 
 		  A[5]|=(B[3]&0x02)<<5;
		  A[5]|=(B[2]&0x20)<<2;
		  A[5]|=(B[3]&0x20)>>3;
			
			
			A[6]|=(B[0]&0x20)<<1;
		  A[6]|=(B[4]&0x20)<<2;
		  A[6]|=(B[7]&0x20)>>3;
 
 			A[7]|=(B[5]&0x20)<<1;
		  A[7]|=(B[6]&0x20)<<2;
		  A[7]|=(B[4]&0x08)>>1;
			
				A[8]|=(B[5]&0x08)<<3;
		  A[8]|=(B[6]&0x08)<<4;
		  A[8]|=(B[7]&0x08)>>1;
						
			A[9]|=(B[3]&0x08)<<3;
		  A[9]|=(B[2]&0x08)<<4;
		  A[9]|=(B[1]&0x08)>>1;
			
			A[10]|=(B[0]&0x08)<<3;
		  A[10]|=(B[3]&0x10)<<3;
		  A[10]|=(B[2]&0x10)>>2;
			
						A[11]|=(B[1]&0x10)<<2;
		  A[11]|=(B[0]&0x10)<<3;
		  A[11]|=(B[4]&0x10)>>2;
			
			A[12]|=(B[5]&0x10)<<2;
		  A[12]|=(B[6]&0x10)<<3;
		  A[12]|=(B[7]&0x10)>>2;
			
			A[13]|=(B[3]&0x40);
		  A[13]|=(B[2]&0x40)<<1;
		  A[13]|=(B[1]&0x40)>>4;
			
			A[14]|=(B[4]&0x40)>>6;
 	
 
				 		 wating();
delay(duty/3+5);
	 GPIOA->ODR=0x00; 
		 for (i=0;i<8;i++){
		delay(2);
			 B[i]=(GPIOB->IDR) ;
			  	GPIOA->ODR+=0x10; 
			 
		 }
		 
		  A[0]|=(B[3]&0x01)<<5;
		 A[0]|=(B[2]&0x01)<<1;
		 A[0]|=(B[1]&0x01)<<3;
		 
		 A[1]|=(B[0]&0x01)<<5;
		 A[1]|=(B[4]&0x01)<<1;
		 A[1]|=(B[5]&0x01)<<3;
		 
		 
		 A[2]|=(B[6]&0x01)<<5;
		 A[2]|=(B[7]&0x01)<<1;
		 A[2]|=(B[4]&0x02)<<2;
		 
		   A[3]|=(B[2]&0x02)<<4;
		  A[3]|=(B[1]&0x02)<<0;
		  A[3]|=(B[0]&0x02)<<2;
		 
		  A[4]|=(B[5]&0x02)<<4;
		  A[4]|=(B[6]&0x02)<<0;
		  A[4]|=(B[7]&0x02)<<2;
		 
 	  A[5]|=(B[3]&0x02)<<4;
		  A[5]|=(B[2]&0x20)>>4;
		  A[5]|=(B[3]&0x20)>>2;

			A[6]|=(B[0]&0x20);
		  A[6]|=(B[4]&0x20)>>4;
		  A[6]|=(B[7]&0x20)>>2;
			
			A[7]|=(B[5]&0x20);
		  A[7]|=(B[6]&0x20)>>4;
		  A[7]|=(B[4]&0x08);
					
			A[8]|=(B[5]&0x08)<<2;
		  A[8]|=(B[6]&0x08)>>2;
		  A[8]|=(B[7]&0x08);
			
				A[9]|=(B[3]&0x08)<<2;
		  A[9]|=(B[2]&0x08)>>2;
		  A[9]|=(B[1]&0x08) ;
			
				A[10]|=(B[0]&0x08)<<2;
		  A[10]|=(B[3]&0x10)>>3;
		  A[10]|=(B[2]&0x10)>>1;
 
			A[11]|=(B[1]&0x10)<<1;
		  A[11]|=(B[0]&0x10)>>3;
		  A[11]|=(B[4]&0x10)>>1;
			
			A[12]|=(B[5]&0x10)<<1;
		  A[12]|=(B[6]&0x10)>>3;
		  A[12]|=(B[7]&0x10)>>1;
			
			A[13]|=(B[3]&0x40)>>1;
		  A[13]|=(B[2]&0x40)>>5;
		  A[13]|=(B[1]&0x40)>>3;
			
			A[14]|=(B[4]&0x40)>>5;
			
				 		 wating();
delay(duty*2/3+5);
	 GPIOA->ODR=0x00; 
		 for (i=0;i<8;i++){
		delay(2);
			 B[i]=(GPIOB->IDR) ;
			  	GPIOA->ODR+=0x10; 
			 
		 }
		  A[0]|=(B[3]&0x01)<<0;
		 A[0]|=(B[2]&0x01)<<4;
		
	  A[1]|=(B[0]&0x01)<<0;
		 A[1]|=(B[4]&0x01)<<4;

		 A[2]|=(B[6]&0x01)<<0;
		 A[2]|=(B[7]&0x01)<<4;
		 
		   A[3]|=(B[2]&0x02)>>1;
		  A[3]|=(B[1]&0x02)<<3;
		 
		 
      A[4]|=(B[5]&0x02)>>1;
		  A[4]|=(B[6]&0x02)<<3;

		   A[5]|=(B[3]&0x02)>>1;
		  A[5]|=(B[2]&0x20)>>1;
 
			A[6]|=(B[0]&0x20)>>5;
		  A[6]|=(B[4]&0x20)>>1;
		 
 			A[7]|=(B[5]&0x20)>>5;
		 A[7]|=(B[6]&0x20)>>1;

			A[8]|=(B[5]&0x08)>>3;
		  A[8]|=(B[6]&0x08)<<1;
 
 				A[9]|=(B[3]&0x08)>>3;
		  A[9]|=(B[2]&0x08)<<1;
	 
 
 			A[10]|=(B[0]&0x08)>>3;
		  A[10]|=(B[3]&0x10);
		
			A[11]|=(B[1]&0x10)>>4;
		  A[11]|=(B[0]&0x10) ;
			
			A[12]|=(B[5]&0x10)>>4;
		  A[12]|=(B[6]&0x10) ;
			
				A[13]|=(B[3]&0x40)>>6;
		  A[13]|=(B[2]&0x40)>>2;
			
			A[14]|=(B[4]&0x40)>>4;
			sum=0;
			for (i=0;i<14;i++){
				if((A[i]&0x01)==0)j=i;
				 switch (A[i]|0x01){
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
				sum+=x1*pow(10,i);
			}
			sumatinon[s]=sum;
			sum=0;
			if(s>0){
				if(sumatinon[s]!=sumatinon[s-1])s--;
			}
			
		}
			sum=sumatinon[2]/pow(10,j);
			

		//beep

		
		
		///rial or toman
		
//		 		pa1=(A[0]&(0x80))>>7;
//			pa1|=(A[0]&(0x2));
//			pa1|=(A[0]&(0x10))>>2;
//		sprintf(buf, "pa1:%x\n",pa1);
//  HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000); 
//			pa2=(A[0]&(0x04))>>2;
//			pa2|=(A[0]&(0x08))>>2;
//			pa2|=0x04;
//		sprintf(buf, "pa2:%x\n",pa2);
//  HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000); 
//		
//				sprintf(buf, "pa2:%x\n",A[14]);
//  HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000); 
//		if(A[14]==pa1){
//					sum=sum*10000;
//		}
//		else if(A[14]==pa2){
//					sum=sum*10;
//		}
sum=sum*m;
//	for(i=0;i<500;i++){
//wating();	
//	 
//		 	GPIOA->ODR=0x40; 
//		delay(3);
//		pa3=(GPIOB->IDR)&0x40;
//			GPIOA->ODR=0x20; 
//				delay(3);
//		pa1=(GPIOB->IDR)&0x00;
//				GPIOA->ODR=0x10; 
//				delay(3);
//		pa1=(GPIOB->IDR)&0x00;
//		
 
		//	GPIOA->ODR=0x40; 
//while(1){
// 			 	 sprintf(buf, "q%x\n",(GPIOB->IDR)&0x40);
//  HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000); 
//}
// 
//	
	
//		  	 sprintf(buf, "sum:%f\n", read());
//  HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
	
	/*
					wating();
	sum1=read();
		sum=sum1;
		 //sum2=read();
//		 if(sum1==sum2){
//			 
//			 sum=sum1;
//}
//		 else{
//			 sum=read();
//		 }
// for(i=0;i<100;i++)buf[i]=0;
//	 sprintf(buf,"sum==x:%f\n" ,sum);
// HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
 */
 for( i=0;i<50;i++){
								buf5[i-12]=0;
								}
  sprintf(buf5,"%0.f",sum);
		 for( i=0;i<14;i++){
								arduino[i]=0;
								}	
//pos pax ethernet
								/*
							 sprintf(arduino,"A%0.fB",sum);//pos pax ethernet
								HAL_UART_Transmit(&huart1, (uint8_t*)arduino,  COUNTOF(arduino)-1, 5000);	
								*/
				//pos pax rs232	
 for( i=0;i<250;i++){
								bufpax[i]=0;
								}								
														 sprintf(bufpax,"%0.f",sum);
							int len =strlen(bufpax);							
						//pos_1	sprintf(hex2,"54454A415241545F42414E4B303230303030303030303034303%c3%c3%c3%c3%c3%c3%c3%c3%c3%c3%c",buf5[len-18],buf5[len-17],buf5[len-16],buf5[len-15],buf5[len-14], buf5[len-13],buf5[len-12],buf5[len-11],buf5[len-10],buf5[len-9],buf5[len-8] );
/*pos_mapna_rs232	*/
			/*			 sprintf(hex2,"54454A415241545F42414E4B303230303030303030303034303%c3%c3%c3%c3%c3%c3%c3%c3%c3%c3%c",buf5[len-18],buf5[len-17],buf5[len-16],buf5[len-15],buf5[len-14], buf5[len-13],buf5[len-12],buf5[len-11],buf5[len-10],buf5[len-9],buf5[len-8] );
							char asciiStr[100];
							for(int i = 0; i < 151; i +=2)
							{
								asciiStr[i/2] = getHexVal(hex2[i])*16 + getHexVal(hex2[i+1]);
								HAL_UART_Transmit(&huart1, (uint8_t*)(asciiStr+i/2), 1, 5000);
							} */
	/*pos_pax_rs232	_a80*/
		/*						
						 //sprintf(hex2,"54454A415241545F42414E4B303230303030303030303034303%c3%c3%c3%c3%c3%c3%c3%c3%c3%c3%c",buf5[len-18],buf5[len-17],buf5[len-16],buf5[len-15],buf5[len-14], buf5[len-13],buf5[len-12],buf5[len-11],buf5[len-10],buf5[len-9],buf5[len-8] );
							if(len<10){
								sprintf(bufpax,"027400000001726EB11B810%d",len);
							}
							else{
									sprintf(bufpax,"027400000001726EB11B81%d",len);
							}
							char part1[250];
							for(int i = 0; i < 24; i +=2)
							{
								part1[i/2] = getHexVal(bufpax[i])*16 + getHexVal(bufpax[i+1]);
								HAL_UART_Transmit(&huart1, (uint8_t*)(part1+i/2), 1, 5000);
							}
						 
			  sprintf(buf5,"%0.f",sum);				
									HAL_UART_Transmit(&huart1, (uint8_t*)buf5,  len, 5000);	
		sprintf(bufpax,"820AD4E4C7D3E53A3130300A830101840103850100B24FA11B810431303030820130830130840130850130860130870130880130A230A10C8105D4E4C7D3E58203313030A1118106446C6C5665728207322E312E302E31A10D81065072675665728203312E3031C1E704");			
							for(int i =0; i < 215; i +=2)	
							{
								part1[i/2] = getHexVal(bufpax[i])*16 + getHexVal(bufpax[i+1]);
								HAL_UART_Transmit(&huart1, (uint8_t*)(part1+i/2), 1, 5000);
							}
							*/
/*pos_pax_rs232_s800	*/
char part1[250];
						sprintf(bufpax,"0017010430363030060c");
						for(int i =0; i < 20; i +=2)	
							{
								part1[i/2] = getHexVal(bufpax[i])*16 + getHexVal(bufpax[i+1]);
								HAL_UART_Transmit(&huart1, (uint8_t*)(part1+i/2), 1, 5000);
							}
							 sprintf(buf5,"%0.f",sum);				
									HAL_UART_Transmit(&huart1, (uint8_t*)buf5, 12, 5000);	
	sprintf(bufpax,"0d0130");
						for(int i =0; i < 6; i +=2)	
							{
								part1[i/2] = getHexVal(bufpax[i])*16 + getHexVal(bufpax[i+1]);
								HAL_UART_Transmit(&huart1, (uint8_t*)(part1+i/2), 1, 5000);
							}
							
 	 	GPIOA->ODR=0x80; 
	 delay(1000);
				 	GPIOA->ODR=0x00; 
	 }
	 }
 else if (cont>100000){
	 cont=0;
	 pa1=0;
	 pa2=0;
	 A[14]=0;
	 			 GPIOA->ODR=0x10; 
			 			  wating();

duty=0;
	while((GPIOB->IDR)&(0x20)){
		delay(1);
		duty++;
		 
	 }
	 	 wating();
		 GPIOA->ODR=0x00; 
		 for (i=0;i<8;i++){
		delay(2);
			 B[i]=(GPIOB->IDR);
			  	GPIOA->ODR+=0x10; 
			 
		 }
		  GPIOA->ODR=0x00; 
	
		pa1=(B[2]&0x01);
		pa2=(B[1]&0x01);  
			A[14]=(B[4]&0x40)>>6;
 
				 		 wating();
delay(duty/3+5);
	 GPIOA->ODR=0x00; 
		 for (i=0;i<8;i++){
		delay(2);
			 B[i]=(GPIOB->IDR) ;
			  	GPIOA->ODR+=0x10; 
			 
		 }
		 
 GPIOA->ODR=0x00; 
		 pa1|=(B[2]&0x01)<<1;
			
			
			A[14]|=(B[4]&0x40)>>5;

	 		 wating();
delay(duty*2/3+5);
	 GPIOA->ODR=0x00; 
		 for (i=0;i<8;i++){
		delay(2);
			 B[i]=(GPIOB->IDR) ;
			  	GPIOA->ODR+=0x10; 
			 
		 }
		  GPIOA->ODR=0x00; 
		pa1|=(B[2]&0x01)<<2;
		pa2|=(B[1]&0x01)<<2;
			A[14]|=(B[4]&0x40)>>4;
		 

//		 		 sprintf(buf,"pa1:%d//pa2:%d//pa3:%d\n",pa1,pa2,A[14]);
// HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
	lastm=m;
		 wating();
		s=0;
		 	GPIOA->ODR=0x40;
		 for(i=0;i<(2*duty);i++){
			 delay(1);
			 if(((GPIOB->IDR)&(0x40))){
				s++;
			 }
		 }
		 if(s>30){
//			 sprintf(buf,"k:%d....%d\n",s,duty);
// HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
			 s=0;
			 
		 if(pa2!=pa1){
		if(A[14]==pa1){
			m=10;
		}
		else if(A[14]==pa2){
		m=10000;
	}
 
 

		
 }
		 }
		 else{
			 m=1;
		 }
//		 		 sprintf(buf,"m:%f \n",m);
// HAL_UART_Transmit(&huart1, (uint8_t*)buf, COUNTOF(buf)-1, 5000);
		 		if(lastm!=m){
	 if(m==10000){
		 for(i=0;i<1;i++){
			 	 	GPIOA->ODR=0x80; 
	 delay(1000);
				 	GPIOA->ODR=0x00; 
			  delay(2000);
			 
		 }
		 
	 }
	 else if(m==10){
		 for(i=0;i<1;i++){
			 	 	GPIOA->ODR=0x80; 
	 delay(1000);
				 	GPIOA->ODR=0x00; 
			  delay(2000);
			 
		 }
	 }
	 else if (m==1){
		 for(i=0;i<1;i++){
			 	 	GPIOA->ODR=0x80; 
	 delay(500);
				 	GPIOA->ODR=0x00; 
			  delay(1000);
			 
		 }
		 
	 }
		 
		}
}
		 
else{
cont++;
}	
	
 }//end while
	}//end main
 
		
 
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

 
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
    ex: printf("Wrong parameters value: file %s on line %drn", file, line) */
  /* USER CODE END 6 */

}

#endif
int getHexVal(char c)
{
  if(c >= '0' && c<= '9')
    return c - '0';
  else if(c >= 'a' && c<= 'f')
    return c - 'a' + 10;
  else if(c >= 'A' && c<= 'F')
    return c - 'A' + 10;
  else
    return -1;//error
}

void send(char* hex){
	

int hexLen = strlen(hex);
char asciiStr[hexLen/2+1];
for(int i = 0; i < hexLen; i +=2)
{
  asciiStr[i/2] = getHexVal(hex[i])*16 + getHexVal(hex[i+1]);
	HAL_UART_Transmit(&huart1, (uint8_t*)(asciiStr+i/2), 1, 5000);
}

}
/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
