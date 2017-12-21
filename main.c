#include "stm32f4xx.h"

#define DS1621_Address		0X48
#define RTC_ADDRESS			0X68

#define START_CONVERTION   	0xEE
#define READ_TEMPERATURE	0xAA
#define ACCESS_CONFIG 		0xAC
#define RTC_REGISTER		0X00

/***********************CONFIG_CLOCK******************************///
void Clock_Config(){						//HSI CLOCK 16MHz
	RCC->CR		|=	RCC_CR_HSION;			// Enable HSI
	while(!(RCC->CR & RCC_CR_HSIRDY));		// Wait till HSI READY
}
/***********************DELAY*************************************/
void delay(__IO uint32_t nCount){			// Delay Function
	while(nCount--);
}
void SendChar(char Tx){
   while(!(USART6->SR & USART_SR_TXE));  			// wait TXBUFF=1
   USART6->DR=Tx;
}

//***************************************USART SEND CHAR BY CHAR**********************************
void SendTxt(char *Adr)
{
  while(*Adr){
    SendChar(*Adr);
    Adr++;
  }
}
char receive(){
	if((USART6->SR & USART_SR_RXNE)!=0){
		char data = USART6->DR;
		return data;
	}
}
void SendChar_WIFI(char Tx){
   while(!(USART1->SR & USART_SR_TXE));  			// wait TXBUFF=1
   USART1->DR=Tx;
}

//***************************************USART SEND CHAR BY CHAR**********************************
void SendTxt_WIFI(char *Adr)
{
  while(*Adr){
    SendChar_WIFI(*Adr);
    Adr++;
  }
}
char reveive_WIFI(){
	if((USART1->SR & USART_SR_RXNE)!=0){
		char data = USART1->DR;
		return data;
	}
}
void ESP_Receive(char * data){
	int i=0;
	for(i=0;i<3;i++){
		while (!(USART1->SR & USART_SR_RXNE ));
		data[i]= (USART1->DR & 0xFF);
		USART1->SR &= ~(USART_SR_RXNE);
	}
}
/***********************CONFIG_USART******************************/
void USART_Config_FTDI(){
	RCC->AHB1ENR 	|= 	RCC_AHB1ENR_GPIOCEN; 			// Enable clock for GPIOC
	RCC->APB2ENR	|= 	RCC_APB2ENR_USART6EN;   		// Enable clock for USART6
	GPIOC->AFR[0]	 =	0x88000000;  					// enable USART6_TX to PC6 and USART6_RX to PC7
	GPIOC->MODER	|=	GPIO_MODER_MODER6_1;			// configuring the USART6 ALTERNATE function PC6
	GPIOC->MODER	|=	GPIO_MODER_MODER7_1;				// configuring the USART6 ALTERNATE function PC7
	USART6->BRR		 =	0x682;    						// 9600 Baud
	USART6->CR1		|=	USART_CR1_UE |USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE; 	// USART6 enable(c=[TE: Transmitter enable %RE:Receiver enable]2=[RXNEIE:RXNE interrupt enable]2=[UE: USART enable] )
}
void USART_Config_WIFI(){
	RCC->AHB1ENR 	|= 	RCC_AHB1ENR_GPIOAEN; 			// Enable clock for GPIOA
	RCC->APB2ENR	|= 	RCC_APB2ENR_USART1EN;   		// Enable clock for USART1
	GPIOA->AFR[1]	 =	0x00000770;  					// enable USART1_TX to PA9 and USART1_RX to PA10
	GPIOA->MODER	|=	GPIO_MODER_MODER9_1;			// configuring the USART1 ALTERNATE function PA9
	GPIOA->MODER	|=	GPIO_MODER_MODER10_1;				// configuring the USART1 ALTERNATE function PA10
	USART1->BRR		 =	0x682;    						// 9600 Baud
	USART1->CR1		|=	USART_CR1_UE |USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE; 	// USART6 enable(c=[TE: Transmitter enable %RE:Receiver enable]2=[RXNEIE:RXNE interrupt enable]2=[UE: USART enable] )
}
void Config_ADC()//PA0 and PA1 & PA2
{
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN; // Enable ADC1 ,ADC2, ADC3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //clock enable GPIOA
	ADC->CCR = ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2 ; // No DMA, Regular simultaneous mode only

	ADC1->CR2 = ADC_CR2_ADON; // Control Register 2: ADC1 ON
	ADC1->SQR3 = 0; // regular SeQuence Register 3

	ADC2->CR2 = ADC_CR2_ADON; // Control Register 2: ADC2 ON
	ADC2->SQR3 = ADC_SQR3_SQ1_0; // regular SeQuence Register 3

	ADC3->CR2 = ADC_CR2_ADON; // Control Register 2: ADC3 ON
	ADC3->SQR3 = ADC_SQR3_SQ1_1 ; // regular SeQuence Register 3

	GPIOA->MODER |= GPIO_MODER_MODER0_0 |GPIO_MODER_MODER0_1 |GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_0 |GPIO_MODER_MODER2_1 ;//Analog mode PA0 and PA1 & PA2
}
/***********************CONFIG_I2C********************************/
void I2C_Config(void){ //I2C on pins PB10<SCL-PB11<SDA
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOBEN ;// Enable Clock GPIOB
	RCC->APB1ENR 	|= RCC_APB1ENR_I2C2EN;// Enable Clock I2C
	GPIOB->AFR[1]  	|= 0x4400;
	GPIOB->MODER	|= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 ;
	GPIOB->MODER	&= ~(GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0) ;
	GPIOB->OTYPER	|= GPIO_OTYPER_OT_10 | GPIO_OTYPER_OT_11 ;
	GPIOB->PUPDR	&= ~(GPIO_PUPDR_PUPDR10_0 |GPIO_PUPDR_PUPDR10_1|GPIO_PUPDR_PUPDR11_0|GPIO_PUPDR_PUPDR11_0);
	I2C2->CR2		|= I2C_CR2_FREQ_4;
	I2C2->CCR		|= 0x50;
	I2C2->TRISE		|= 0x11;
	I2C2->CR1		|= I2C_CR1_PE; //Enable Peripheral
	SendTxt("CONFIG I2C.......\n\n");
}
/***********************I2C_START*********************************/
void I2C_Start(){
	I2C2->CR1		|=I2C_CR1_START;				 //Enable Start Condition
	while(!(I2C2->SR1 & I2C_SR1_SB));				// wait until Flag UP (Start I2C)
	//SendTxt("...............START I2C\n");
}
/***********************I2C_SEND_ADDRESS*******************************/
void I2C_Send_Address(uint8_t Address, char direction){
	if (direction == 'w') {
			I2C2->DR		=(Address<<1)&0xFE;
			while( !(I2C2->SR1 &  I2C_SR1_ADDR));
			int status2=I2C2->SR2;
			//SendTxt("...............I2C SEND ADDRESS WRITE\n");
		}
		else if (direction == 'r') {
			I2C2->DR = ((Address<<1)|0x1);
			while(!(I2C2->SR1 & I2C_SR1_ADDR));			//wait until ADDR is received && read selected
			//SendTxt("...............I2C SEND ADDRESS READ\n");
		}

}
/**********************I2C_SEND_DATA***********************************/
void I2C_Write (uint8_t data){
	I2C2->DR = data;
	while(!( I2C2->SR1 &I2C_SR1_TXE)); //wait until DR empty
	while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred
	//SendTxt("...............I2C WRITE\n");
}
/**************************I2C_STOP************************************/
void I2C_Stop(){
	I2C2->CR1 |= I2C_CR1_STOP;	// I2C STOP
	//SendTxt("...............I2C STOP\n\n");
}

void  DS1621_Config(){

	I2C_Start();
	//SendTxt("start I2C.......\n\n");
	I2C_Send_Address(DS1621_Address,'w');
	//SendTxt("send adrr I2C.......\n\n");
	I2C_Write(START_CONVERTION);
	I2C_Stop();

	I2C_Start();
	I2C_Send_Address(DS1621_Address,'w');
	I2C_Write(ACCESS_CONFIG);
	I2C_Write(0x08);
	I2C_Stop();
	SendTxt("CONFIG dd1621 done.......\n\n");

}
unsigned char DS1621_Read(){
	I2C_Start();
	I2C_Send_Address(DS1621_Address,'w');
	I2C_Write(READ_TEMPERATURE);
	I2C_Start();
	I2C_Send_Address(DS1621_Address,'r');
	I2C2->CR1	|=I2C_CR1_POS;	//read 2 byte
	int status2=I2C2->SR2;//read register SR2 to reset flag
	while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred
	I2C_Stop();
	uint8_t temp=I2C2->DR;
	int x=I2C2->DR;
	return temp;
}

void RTC_Read(uint8_t temps[]){
	I2C_Start();
	//SendTxt("trc start \n");
	I2C_Send_Address(RTC_ADDRESS,'w');
	//SendTxt("addr sent \n");
	I2C_Write(RTC_REGISTER);
	//SendTxt("reg sent \n");
	I2C_Start();
	I2C_Send_Address(RTC_ADDRESS,'r');
	//SendTxt("sent read \n");
	int i;
		for( i=0;i<=6;i++){
			if (i==6) {
				I2C2->CR1 &= ~ (I2C_CR1_ACK);	//not ackint status2=I2C2->SR2;
				int status2=I2C2->SR2;
				while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred

			}
			else {
				I2C2->CR1 |= I2C_CR1_ACK;	//ack
				int status2=I2C2->SR2;
				while(!(I2C2->SR1 & I2C_SR1_BTF));//wait until byteTrasferred
			}
			temps[i]=I2C2->DR;
		}
	I2C_Stop();
}
void RTC_Set(uint8_t hour,uint8_t min,uint8_t day,uint8_t date,uint8_t month,uint8_t year){
	I2C_Start();
	I2C_Send_Address(RTC_ADDRESS,'w');
	I2C_Write(RTC_REGISTER);
	I2C_Write(0x00);
	I2C_Write(min);
	I2C_Write(hour & 0x3F);
	I2C_Write(day);
	I2C_Write(date);
	I2C_Write(month);
	I2C_Write(year);
	I2C_Stop();
	SendTxt("CONFIG rtc done.......\n\n");
}
unsigned char LM35_Read(){
	ADC1->CR2 |= ADC_CR2_SWSTART; // simultaneous Start Conversion
	while(!(ADC1->SR & 0x2)); // wait for ADC1 conversion to complete
	int temp =	ADC1->DR;
	unsigned char temperature =(temp*100)*(3.3/4095);//4095 ADC 12 bits(2^12-1)
	return temperature;
}
float Voltage_Read(){
	ADC2->CR2 |= ADC_CR2_SWSTART; // simultaneous Start Conversion
	while(!(ADC2->SR & 0x2)); // wait for ADC1 conversion to complete
	int volt =	ADC2->DR;
	float voltage =((volt*6)*(3.0/4095))*(4.65/5.1);//4095 ADC 12 bits(2^12-1)
	return voltage;
}
float Current_Read(){
	ADC3->CR2 |= ADC_CR2_SWSTART; // simultaneous Start Conversion
	while(!(ADC3->SR & 0x2)); // wait for ADC1 conversion to complete
	float amperage =	ADC3->DR;
	float Current =((amperage/(11*0.05))*(3.0/4095))*1000*(12.5/275);//4095 ADC 12 bits(2^12-1)
	return Current;
}
void Config_PWM(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //clock enable for TIM3
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //clock enable GPIOA
	GPIOA->MODER |= GPIO_MODER_MODER6_1; //Alternative function mode PA6
	TIM3->CCMR1 = TIM_CCMR1_OC1M_1 |TIM_CCMR1_OC1M_2; //PWM mode 1 on TIM3 Channel 1
	TIM3->PSC = 2;   // Fc=f/PSC+1    avec  f=16Mhz;; ARR=256;; =Fc/256
	TIM3->ARR = 255;
	TIM3->CCR1 = 0;
	GPIOA->AFR[0] = 0x2000000; //set GPIOA to AF2
	TIM3->CCER	|=TIM_CCER_CC1E;
	//TIM3->EGR	|=TIM_EGR_CC1G;//enable interrupt
	TIM3->CR1 	 = TIM_CR1_CEN; //enable counter of tim3

}
int main(void){
	Clock_Config();
	Config_PWM();
	USART_Config_FTDI();
	USART_Config_WIFI();
	I2C_Config();
	DS1621_Config();
	//RTC_Set(0x15,0x58,0x03,0x13,0x12,0x17);
	Config_ADC();
	SendTxt("WELCOME:\n");

	uint8_t TEMPS[7];
	char Time[100]   ={0};
	char Temperature[100] ={0};
	char Puissance[100]   ={0};
	while(1){
		TIM3->CCR1++;
		if(TIM3->CCR1==TIM3->ARR)
		{
			TIM3->CCR1=0;
		}

		RTC_Read(TEMPS);
		if (TEMPS[2]< 0x10) sprintf(Time,"0%x",TEMPS[2]) ;
		else sprintf(Time,"%x",TEMPS[2]) ;
		SendTxt(Time);
		if (TEMPS[1]< 0x10) sprintf(Time,":0%x",TEMPS[1]) ;
		else sprintf(Time,":%x",TEMPS[1]) ;
		SendTxt(Time);
		if (TEMPS[0]< 0x10) sprintf(Time,":0%x",TEMPS[0]) ;
		else sprintf(Time,":%x ",TEMPS[0]) ;
		SendTxt(Time);
		switch (TEMPS[3]){
			case 1 : SendTxt("Monday    "); break;
			case 2 : SendTxt("Tuesday   "); break;
			case 3 : SendTxt("Wednesday "); break;
			case 4 : SendTxt("Thursday  "); break;
			case 5 : SendTxt("Friday    "); break;
			case 6 : SendTxt("Saturday  "); break;
			case 7 : SendTxt("Sunday    "); break;
		}
		if (TEMPS[4]< 0x10) sprintf(Time,":0%x",TEMPS[4]) ;
		else sprintf(Time," %x",TEMPS[4]) ;
		SendTxt(Time);
		if (TEMPS[5]< 0x10) sprintf(Time,":0%x",TEMPS[5]) ;
		else sprintf(Time,":%x",TEMPS[5]) ;
		SendTxt(Time);
		if (TEMPS[6]< 0x10) sprintf(Time,":200%x \n\n",TEMPS[6]) ;
		else sprintf(Time,":20%x \n\n",TEMPS[6]) ;
		SendTxt(Time);

		sprintf(Temperature,"DS1621= %d  ,  LM35= %d\n\n",DS1621_Read(),LM35_Read()) ;
		SendTxt(Temperature);
		sprintf(Puissance,"tension=%f V ,  courant=%f  mA , Puissance= %f mW\n\n",Voltage_Read(), Current_Read(),Voltage_Read()*Current_Read()) ;
		SendTxt(Puissance);
		delay(3000000);
	}
return ;
}
