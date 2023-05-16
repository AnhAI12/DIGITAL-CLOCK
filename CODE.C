/***********
				Author: Le Duy Anh
*/

/*********I2C Pin***********
		PB6 --> SCL
		PB7--> SDA 
		DS3231 address 7 bit: 1101000
*********SPI Pin*********
		* PA4--> NSS, config as output
			PA5-->SCK
			PA6-->MISO
			PA7-->MOSI 
*/

#include "stm32f4xx.h"                  // Device header

#define DS3231_ADDR			0xD0
#define HOUR_ADDR				0x02
#define MINUTE_ADDR			0x01
#define SECOND_ADDR			0x00



typedef struct {
	uint16_t second;
	uint16_t minute;
	uint16_t hour;
}time_t;

volatile uint8_t data_check;

/******* Function SPI********/
uint8_t max7219_code[10] = {0x7E, 0x30, 0x6D, 0x79, 0x33, 0x5B, 0x5F, 0x70, 0x7F, 0x7B};

void init_MAX7219(void);
void init_spi(void);
void spi_gpioInit(void);
void spi_sendData(uint16_t data);
void spi_dataTransmit(uint16_t data);
void delay_ms(uint32_t ms);
void spi_enable(void);
void spi_disable(void);
void ss_enable(void);
void ss_disable(void);
void MAX72119_Display(time_t data_display);



/**********Function I2C******/
void I2C1_Init(void);
void I2C1_Start(void);
void I2C1_Write(uint8_t data);
void I2C1_sendAddr(uint8_t address);
void I2C1_stop(void);
void I2C1_receive(uint8_t addr, uint8_t* data);
void DS3231_Read(uint8_t reg, uint8_t* data);
void Read_RealTime(uint8_t *data, time_t* pt);

void delay_ms(uint32_t ms)
{
	for (uint32_t i = 0; i < ms; i++)
	{
		for (uint32_t j = 0; j < 5000; j++);
	}
}

/******* MAIN FUNCTION**************************************************/
int main(void)
{
	I2C1_Init();
	spi_gpioInit();
	init_spi();
	
	SPI1->CR1 |= 1<<6;		//enable SPE
	init_MAX7219();
//	spi_dataTransmit( (1<<8)|0x7E);
//	spi_dataTransmit( (2<<8)|max7219_code[1]);
//	spi_dataTransmit( (3<<8)|max7219_code[2]);
//	spi_dataTransmit( (4<<8)|max7219_code[3]);
//	spi_dataTransmit( (5<<8)|max7219_code[4]);
//	spi_dataTransmit( (6<<8)|max7219_code[5]);
//	spi_dataTransmit( (7<<8)|max7219_code[5]);
	spi_dataTransmit( (8<<8)|max7219_code[0]);
	
	uint8_t data;
	volatile uint16_t data2;
	time_t data_time;
	data_time.second=0;
	data_time.minute=0;
	data_time.hour=0;
	
	while(1){
		delay_ms(10);
//		DS3231_Read(0x00, &data);
//		data_time.minute=data;
		Read_RealTime(&data, &data_time);
		MAX72119_Display(data_time);
	}
}



/******************** I2C1 & DS3231 *************************/

void I2C1_Init(void){
	/********* Config GPIOB ********/
	/* PB6 --> SCL
		PB7--> SDA 
		Pin I2C must be a Open-Drain + Pull-up*/
	//ENABLE RCC GPIOB
	RCC->AHB1ENR |= 1<<1;
	//Alternate for PB6 PB7
	GPIOB->MODER |= (1<<15)|(1<<13);
	// AF4-->I2C
	GPIOB->AFR[0] |= (4<<28)|(4<<24);
	//Output High Speed
	GPIOB->OSPEEDR |= (1<<15)|(1<<13);
	//Output Open-drain
	GPIOB->OTYPER |= 3<<6;
	// Pull-up Both pin
	GPIOB->PUPDR |= (1<<14)|(1<<12);
	
	/***********Config I2C MASTER MODE *********/
	/*Program the peripheral input clock in I2C_CR2 register in order to generate correct
		timings
		• Configure the clock control registers
		• Configure the rise time register
		• Program the I2C_CR1 register to enable the peripheral
		• Set the START bit in the I2C_CR1 register to generate a Start condition
		The peripheral input clock frequency must be at least:
		• 2 MHz in Sm mode
		 4 MHz in Fm mode */
	
	//Reset I2C
	I2C1->CR1 |= (1<<15);
	I2C1->CR1 &= ~(1<<15);

	//enable RCC I2C1
	RCC->APB1ENR |= 1<<21;
	//FREQ = APB1 CLOCK = 16Mhz
	I2C1->CR2 |= 16<<0;
	//Standard mode 100Khz, Sm mode
	//I2C1->CCR &= ~(1<<15);
	//Clock Control 100Khz --> RCC*x=5000ns
	I2C1->CCR |= 80<<0;
	//the maximum allowed SCL rise time is 1000 ns (t_rise(max)),--> 1000 ns / 62.5 ns +1 = 16+1 --> TRISE[0:5]= 17
	I2C1->TRISE =17;
	//ENABLE PERIPHERAL
	I2C1->CR1 |= 1<<0;
//	// NO repeat START condition
//	I2C1->CR1 &=~(1<<8);
	//
	
}

void I2C1_Start(void){
	/******
	1. Send START condition
	2. wait for the SB (SR1) to set. this bit indicates that START condition is generated */
	// NO repeat START condition
	I2C1->CR1 |= (1<<10);	//set THE ACK bit
	I2C1->CR1 |=(1<<8); //generate START
	while( !(I2C1->SR1 & (1<<0) ) );		//Wait SB=1
	
}

void I2C1_Write(uint8_t data){
	/*********
	1. wait for the TXE (SR1) to set. This indicates that the DR is empty
	2. Send the DATA to the DR register
	3. Wait for the BTF (SR1) to set. The end of LAST DATA transmission */
	
	while( !( I2C1->SR1 & (1<<7) ) ); //wait TXEE bit =1
	I2C1->DR = data;
	while(! (I2C1->SR1 & (1<<2) ));	//wait BTF bit =1
	
}

void I2C1_sendAddr(uint8_t address){
	/***** 
	1. Send the slave Address to the DR register
	2. Wait for the ADDR( SR1) to set. The end of address transmission
	3. Clear the ADDR by reading SR1 & SR2 */
	I2C1->DR = address;		//Send the address
	while( !(I2C1->SR1 & (1<<1)));	//Wait ADDR bit =1
	uint8_t temp= I2C1->SR1 | I2C1->SR2;	//read SR1 & SR2 to clear ADDR bit
	
}

void I2C1_stop(void){
	I2C1->CR1 |=(1<<9);		//Stop I2C
}

void I2C1_receive(uint8_t addr, uint8_t* data){
	/***Only 1 byte needs to be read
	1. write the slave address (having bit R/W) + wait for the ADDR bit =1
	2. the ACK disable is made during EV6
	3. wait for the RXNE bit =1 (Receive not empty)
	4. Read the data from the DR */
	
	I2C1->DR = (addr);		//send address
	while( !( I2C1->SR1 & (1<<1)));	//wait for ADDR bit=1
	I2C1->CR1 &= ~(1<<10);	//CLEAR THE ACK bit
	uint8_t temp = I2C1->SR1 | I2C1->SR2;	//READ_BIT to clear ADDR
	I2C1->CR1 |= (1<<9);
	
	while(!(I2C1->SR1 & (1<<6)));	//wait for RxNE to set
	*data = I2C1->DR;	//READ the data
}

void DS3231_Read(uint8_t reg, uint8_t* data)
{
	I2C1_Start();
	I2C1_sendAddr(DS3231_ADDR);
	I2C1_Write(reg);	//writing register needs read
	I2C1_Start();			//repeat start condition
	I2C1_receive(DS3231_ADDR+0x01, data);
	I2C1_stop();
	
	*data = (*data>>4)*10+ (*data&0x0F);
}

void Read_RealTime(uint8_t *data, time_t* pt){
	DS3231_Read(SECOND_ADDR, data);
	pt->second=*data;
	DS3231_Read(MINUTE_ADDR, data);
	pt->minute=*data;
	DS3231_Read(HOUR_ADDR, data);
	pt->hour=*data;
}

/**************** SPI & MAX7219 ***********************************************/
void spi_enable()
{
	SPI1->CR1 |= 1<<6;
}

void spi_disable()
{
	SPI1->CR1 &= ~(1<<6);
}

void ss_enable(void)
{
	GPIOA->BSRR |= (1<<4)<<16;		//reset bit, PA4=0
}

void ss_disable(void)
{
	GPIOA->BSRR |= (1<<4);		//set bit, PA4=1
}
void spi_gpioInit(void){
	/* PA4--> NSS, config as output
			PA5-->SCK
			PA6-->MISO
			PA7-->MOSI */
	RCC->AHB1ENR |= 1;															//enable GPIOA
	GPIOA->MODER |= (1<<15)|(1<<13)|(1<<11)|(1<<8);	//alternate function,PA7,PA5,PA6,output PA4
	//GPIOA->OTYPER &=~(15<<4);												//PUSH-PULL
	//GPIOA->AFR[0] =0x55550000;											//AF5 SPI
	GPIOA->AFR[0] |= (5<<20)|(5<<24)|(5<<28);		//AF5 SPI
	GPIOA->OSPEEDR |= (1<<15)|(1<<13)|(1<<11)|(1<<9);		//HIGH SPEED for PA...
}

void init_spi(void){
	//enable rcc
	RCC->APB2ENR |= 1<<12;	//SPI1 ENABLE
	SPI1->CR1 |= 1<<2;			// Master configuration
	//
	SPI1->CR1 |= (1<<3);		//baud rate = fPCLk/4 =4Mhz, fSYSCLK = 16Mhz
	SPI1->CR1 &= ~(3);			//CPOL=CPHA=0;
	SPI1->CR1 |= 1<<11;			//bit DFF=1, 16-bit data frame
	SPI1->CR1 &= ~(1<<7);		//bit MSB first
	//SPI1->CR2 |= 1<<2;			//bit SSOE, NSS output
	SPI1->CR1 |= (1<<9);		//SSM =1 -->software slave management
	SPI1->CR1 |= 1<<8;			//SSI=1
	//SPI1->CR1 &= ~(1<<10);	//Full duplex, trans and recei
}

void spi_sendData(uint16_t data){
	//reset bit NSS
	//SPI1->CR1 &=~ (1<<8);	
	//1. wait until TXE is set
	while(! (SPI1->SR & (1<<1)));
	SPI1->DR = data;
	//wait BSY=0, not busy
	while(! (SPI1->SR & (1<<1)));
	while((SPI1->SR & (1<<7)));
	
//	uint16_t temp = SPI1->DR;
//		temp = SPI1->SR;
}


void init_MAX7219(){
	// decode mode: no decode mode for all digits
	//sendata(0x09, 0x00);
	//spi_sendData( (0x09<<8)|0x00);
	spi_dataTransmit((0x09<<8)|0x00);
	// intensiy
	//sendata(0x0A, 0x09);
	//spi_sendData( (0x0A<<8)|0x07);
	spi_dataTransmit((0x0A<<8)|0x07);
	// scan limit
	//sendata(0x0B, 0x07);
	//spi_sendData( (0x0B<<8)|0x07);
	spi_dataTransmit((0x0B<<8)|0x07);
	// not shutdown, turn off display text
	//sendata(0x0C, 0x01);
	//spi_sendData( (0x0C<<8)|0x01);
	spi_dataTransmit((0x0C<<8)|0x01);
	//sendata(0x0F, 0x00);
	spi_dataTransmit((0x0F<<8)|0x00);
}

void spi_dataTransmit(uint16_t data){
	ss_enable();
	spi_sendData(data);
	ss_disable();
}

void MAX72119_Display(time_t data_display){
	spi_dataTransmit( (1<<8)|max7219_code[data_display.second%10]);
	spi_dataTransmit( (2<<8)|max7219_code[data_display.second/10]);
	spi_dataTransmit( (3<<8)|0x80);
	spi_dataTransmit( (4<<8)|max7219_code[data_display.minute%10]);
	spi_dataTransmit( (5<<8)|max7219_code[data_display.minute/10]);
	spi_dataTransmit( (6<<8)|0x80);
	spi_dataTransmit( (7<<8)|max7219_code[data_display.hour%10]);
	spi_dataTransmit( (8<<8)|max7219_code[data_display.hour/10]);
}
