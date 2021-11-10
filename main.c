#include "MKL25Z4.h"                    // Device header

#define BUS_CLOCK ( 24000000 )

void vInitializeUART1( uint32_t baud_rate );
	
int main( void )
{
	uint8_t u8Data[9] = {0x68, 0x65, 0x79, 0x20, 0x74, 0x68, 0x65, 0x72, 0x65};
	while( 1 )
	{
		for( int i = 0; i < 9; i++ )
		{
			if( UART0->S1 & UART_S1_TDRE_MASK )
			{
				// transmit data register is empy, can write more data to data register
				UART1->D = u8Data[i];
			}
		}
		
		for( int j = 0; j < 65535; j++ )
		{
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		}
		
	}
}

void vInitializeUART1( uint32_t baud_rate )
{
	uint32_t divisor;
	// enable clock to UART and Port A
	SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// connect UART to pins for PTE22, PTE23
	PORTE->PCR[22] = PORT_PCR_MUX(4);
	PORTE->PCR[23] = PORT_PCR_MUX(4);
	// ensure tx and rx are disabled before configuration
	UART2->C2 &= ~(UARTLP_C2_TE_MASK | UARTLP_C2_RE_MASK);
	// Set baud rate to 4800 baud
	divisor = BUS_CLOCK/(baud_rate*16);
	UART2->BDH = UART_BDH_SBR(divisor>>8);
	UART2->BDL = UART_BDL_SBR(divisor);
	// No parity, 8 bits, two stop bits, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;
	// Enable transmitter and receiver
	UART2->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
}
