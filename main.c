#include "MKL25Z4.h"                    // Device header

#define BUS_CLOCK ( 10526316 )

// PORT A
#define SENSOR_0_GREEN_PIN ( 17 )
#define SENSOR_0_YELLOW_PIN ( 16 )
#define SENSOR_0_RED_PIN ( 13 )

#define SENSOR_1_GREEN_PIN ( 4 )
#define SENSOR_1_YELLOW_PIN ( 12 )
#define SENSOR_1_RED_PIN ( 5 )

#define MASK(x) (1UL << (x))

void vInitializeUART2(uint32_t baud_rate) ;
void vInitializeLEDs( void );
uint8_t UART2_Receive_Poll( void );
void UART2_Transmit_Poll(uint8_t data);
	
int main( void )
{
	char charBuf[3];
	uint8_t u8CharsRxd = 0;
	//uint8_t u8Data[9] = {0x68, 0x65, 0x79, 0x20, 0x74, 0x68, 0x65, 0x72, 0x65};
	vInitializeLEDs();
	vInitializeUART2( 4800 );
	
	while( 1 )
	{
		
//		UART2_Transmit_Poll('H');
//		UART2_Transmit_Poll('I');
//		UART2_Transmit_Poll(0x00);
		charBuf[ u8CharsRxd ] = UART2_Receive_Poll();
		
		u8CharsRxd++;
		
		if( u8CharsRxd == 3 )
		{
			if( charBuf[0] == 'H' )
			{
				PTA->PCOR = ( MASK( SENSOR_0_YELLOW_PIN ) | MASK( SENSOR_1_GREEN_PIN ) | MASK( SENSOR_1_RED_PIN ) );
			}
			u8CharsRxd = 0;
		}
		
		
		
		
		
//		PTA->PCOR = ( MASK( SENSOR_0_GREEN_PIN ) | MASK( SENSOR_0_RED_PIN ) | MASK( SENSOR_1_YELLOW_PIN ) );
//		PTA->PSOR = ( MASK( SENSOR_0_YELLOW_PIN ) | MASK( SENSOR_1_GREEN_PIN ) | MASK( SENSOR_1_RED_PIN ) );
//	
//		for( int j = 0; j < 65535; j++ )
//		{
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//		}
//		
//		PTA->PCOR = ( MASK( SENSOR_0_YELLOW_PIN ) | MASK( SENSOR_1_GREEN_PIN ) | MASK( SENSOR_1_RED_PIN ) );
//		PTA->PSOR = ( MASK( SENSOR_0_GREEN_PIN ) | MASK( SENSOR_0_RED_PIN ) | MASK( SENSOR_1_YELLOW_PIN ) );
//		
//		for( int j = 0; j < 65535; j++ )
//		{
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//			__NOP();
//		}
	
	}
}

//void vInitializeUART1( uint32_t baud_rate )
//{
//	uint32_t divisor;
//	// enable clock to UART and Port E
//	SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
//	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
//	
//	// connect UART to pins for PTE22, PTE23
//	PORTE->PCR[22] = PORT_PCR_MUX(4);
//	PORTE->PCR[23] = PORT_PCR_MUX(4);
//	// ensure tx and rx are disabled before configuration
//	UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
//	// Set baud rate to 4800 baud
//	divisor = BUS_CLOCK/(baud_rate*16);
//	UART2->BDH = UART_BDH_SBR(divisor>>8);
//	UART2->BDL = UART_BDL_SBR(divisor);
//	// No parity, 8 bits, two stop bits, other settings;
//	UART2->C1 = UART2->S2 = UART2->C3 = 0;
//	// Enable transmitter and receiver
//	UART2->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
//}

uint8_t UART2_Receive_Poll( void ) 
{
		// wait until receive data register is full
		while( !( UART2->S1 & UART_S1_RDRF_MASK ) );
		return UART2->D;
}

void UART2_Transmit_Poll(uint8_t data) 
{
	// wait until transmit data register is empty
	while ( !( UART2->S1 & UART_S1_TDRE_MASK ) );
	UART2->D = data;
}

void vInitializeUART2(uint32_t baud_rate) 
{
	uint32_t divisor;
	uint32_t u32CLKDIV1 = 0;
	uint32_t u32SBR = 0;
	
	u32CLKDIV1 = SIM_CLKDIV1;
	
	// enable clock to UART and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// connect UART to pins for PTE22, PTE23
	PORTE->PCR[22] = PORT_PCR_MUX(4);
	PORTE->PCR[23] = PORT_PCR_MUX(4);
	
	// ensure tx and rx are disabled before configuration
	UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
	
	// Set baud rate to 4800 baud
	divisor = ( BUS_CLOCK / ( baud_rate * 16 ) );
	UART2->BDH = UART_BDH_SBR(divisor>>8); // UART_BDH_SBR( 0 );
	UART2->BDL = UART_BDL_SBR(divisor); //UART_BDL_SBR( 1 );
	
//	u32SBR = UART2->BDL;
//	u32SBR |= ( ( UART2->BDH & 0x1FU ) << 8U );
	
	// No parity, 8 bits, one stop bit, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;
	
	// Enable transmitter and receiver
	UART2->C2 = ( UART_C2_TE_MASK | UART_C2_RE_MASK );
}


void vInitializeLEDs( void )
{
	// enable the clock to port A
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	// configure PORT A and PORT B pins as GPIO
	PORTA->PCR[SENSOR_0_GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SENSOR_0_GREEN_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[SENSOR_0_YELLOW_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SENSOR_0_YELLOW_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[SENSOR_0_RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SENSOR_0_RED_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[SENSOR_1_GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SENSOR_1_GREEN_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[SENSOR_1_YELLOW_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SENSOR_1_YELLOW_PIN] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[SENSOR_1_RED_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SENSOR_1_RED_PIN] |= PORT_PCR_MUX(1);
	
	// set pins connected to LEDs as outputs
	PTA->PDDR |= ( MASK(SENSOR_0_GREEN_PIN) | MASK(SENSOR_0_YELLOW_PIN) | MASK(SENSOR_0_RED_PIN) | 
	MASK(SENSOR_1_GREEN_PIN) | MASK(SENSOR_1_YELLOW_PIN) | MASK(SENSOR_1_RED_PIN)	);
}
