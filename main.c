#include "MKL25Z4.h"                    // Device header

#define BUS_CLOCK ( 10526316 )

// PORT A
#define SENSOR_0_GREEN_PIN ( 17 )
#define SENSOR_0_YELLOW_PIN ( 16 )
#define SENSOR_0_RED_PIN ( 13 )

#define SENSOR_1_GREEN_PIN ( 4 )
#define SENSOR_1_YELLOW_PIN ( 12 )
#define SENSOR_1_RED_PIN ( 5 )

//Sensor 0 LEDs on port E
#define SENSOR_0_PORTE_LED0 ( 0 ) // lowest moisture setting
#define SENSOR_0_PORTE_LED1 ( 1 )
#define SENSOR_0_PORTE_LED2 ( 2 )
#define SENSOR_0_PORTE_LED3 ( 3 )
#define SENSOR_0_PORTE_LED4 ( 4 ) // highest moisture setting

// Sensor 1 LEDs on port E
#define SENSOR_1_PORTE_LED0 ( 0 ) // lowest moisture setting
#define SENSOR_1_PORTE_LED1 ( 1 )
#define SENSOR_1_PORTE_LED2 ( 2 )
#define SENSOR_1_PORTE_LED3 ( 3 )
#define SENSOR_1_PORTE_LED4 ( 4 ) // highest moisture setting

#define SENSOR_COUNT ( 2 )
#define LED_COUNT_PER_SENSOR ( 5 )

#define LOWEST_VALID_CAPACITANCE ( 150 )
#define HIGHEST_VALID_CAPACITANCE ( 2050 )

#define LOWEST_VALID_TEMPERATURE ( -30 )
#define HIGHEST_VALID_TEMPERATURE ( 120 )

#define MASK(x) (1UL << (x))

static void vInitializeUART2(uint32_t baud_rate) ;
static void vInitializeLEDs( void );
static uint8_t UART2_Receive_Poll( void );
static void UART2_Transmit_Poll(uint8_t data);
static void vUpdateDisplay( uint8_t u8WhichSensor );

typedef struct xSensorStatusType
{
	uint16_t u16Cap;
	int8_t s8Temp;
	int8_t discard;
} xSENSOR_STATUS_TYPE;

static xSENSOR_STATUS_TYPE xSensorStatus[SENSOR_COUNT];

static const uint8_t u8SensorLEDCfg [SENSOR_COUNT][LED_COUNT_PER_SENSOR] = 
{
	{  2,  3,  4,  5, 20 },
	{ 21, 22, 23, 29, 30 }
};
	
int main( void )
{
	uint8_t u8Buf[8];
	uint8_t u8BytesRxd = 0;
	//uint8_t u8Data[9] = {0x68, 0x65, 0x79, 0x20, 0x74, 0x68, 0x65, 0x72, 0x65};
	vInitializeLEDs();
	vInitializeUART2( 4800 );
	uint16_t u16Cap;
	int8_t s8Temp;
	
	while( 1 )
	{
		
//		UART2_Transmit_Poll('H');
//		UART2_Transmit_Poll('I');
//		UART2_Transmit_Poll(0x00);
		u8Buf[ u8BytesRxd ] = UART2_Receive_Poll();
		
		u8BytesRxd++;
		
		if( u8BytesRxd >= 8 )
		{
			// received the entire payload, now parse to figure out what kind of reading it is
			
			if( u8Buf[0] < SENSOR_COUNT )
			{
				switch( u8Buf[1] )
				{
					case ( 0x00 ):
						// received capacitance reading
						u16Cap = (uint16_t)( ( ( (uint16_t)u8Buf[2] ) << 8U ) | (uint16_t)u8Buf[3] );
					
						if( u16Cap >= LOWEST_VALID_CAPACITANCE && u16Cap <= HIGHEST_VALID_CAPACITANCE )
						{
							xSensorStatus[u8Buf[0]].u16Cap = u16Cap;
							vUpdateDisplay( u8Buf[1] );
						}
						
					break;
					
					case ( 0x01 ):
					// received temperature reading
					s8Temp = (int8_t)u8Buf[2];
					
					if( s8Temp >= LOWEST_VALID_TEMPERATURE && s8Temp <= HIGHEST_VALID_TEMPERATURE )
					{
						// valid temperature
						xSensorStatus[u8Buf[0]].s8Temp = s8Temp;
						vUpdateDisplay( u8Buf[1] );
					}
						
					break;
				}
				
			}
			
			u8BytesRxd = 0;
			
		}
	}
}

static void vUpdateDisplay( uint8_t u8WhichSensor )
{
	int i;
	uint16_t u16Cap;
	uint32_t u32TurnOffLEDMask = 0;
	
	// first turn off all the LEDs for the sensor
	
	for( i = 0; i < LED_COUNT_PER_SENSOR; i++ )
	{ 
		// active low LEDs, so set high to turn them off
		u32TurnOffLEDMask |= ( MASK( u8SensorLEDCfg[u8WhichSensor][i] ) );
	}
	
	PTE->PSOR = u32TurnOffLEDMask;
	
	// LEDs for u8WhichSensor should now all be off
	
	u16Cap = xSensorStatus[u8WhichSensor].u16Cap;
	
	if( u16Cap < 350 )
	{
		// blink LED0
		
	}
	else if( ( u16Cap >= 350 ) && ( u16Cap < 700 ) )
	{
		// light LED 0
		// first clear all of the LEDs
		PTE->PCOR = MASK( u8SensorLEDCfg[u8WhichSensor][0] );
	}
	else if( ( u16Cap >= 700 ) && ( u16Cap < 1000 ) )
	{
		// light LED 0 and 1
		PTE->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) );
	}
	else if( ( u16Cap >= 1000 ) && ( u16Cap < 1250 ) )
	{
		// light LED 0, 1 and 2
		PTE->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) | 
		MASK( u8SensorLEDCfg[u8WhichSensor][2] ) );
	}
	else if( ( u16Cap >= 1250 ) && ( u16Cap < 1500 ) )
	{
		// light LED 0, 1, 2, 3
		PTE->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) | 
		MASK( u8SensorLEDCfg[u8WhichSensor][2] ) | MASK( u8SensorLEDCfg[u8WhichSensor][3] ) );
	}
	else if( ( u16Cap >= 1500 ) && ( u16Cap <= 1750 ) )
	{
		// light LED 0, 1, 2, 3 and 4
		PTE->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) | 
		MASK( u8SensorLEDCfg[u8WhichSensor][2] ) | MASK( u8SensorLEDCfg[u8WhichSensor][3] ) | MASK( u8SensorLEDCfg[u8WhichSensor][4] ) );
	}
	else
	{
		// blink LED 0, 1, 2, 3, and 4
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

static uint8_t UART2_Receive_Poll( void ) 
{
		// wait until receive data register is full
		while( !( UART2->S1 & UART_S1_RDRF_MASK ) );
		return UART2->D;
}

static void UART2_Transmit_Poll(uint8_t data) 
{
	// wait until transmit data register is empty
	while ( !( UART2->S1 & UART_S1_TDRE_MASK ) );
	UART2->D = data;
}

static  void vInitializeUART2(uint32_t baud_rate) 
{
	uint32_t divisor;
	
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
	UART1->C4 &= ~UART_C4_TDMAS_MASK;
	
	I2C1->F |= (I2C_F_MULT(2) | I2C_F_ICR(15) );
}


static void vInitializeLEDs( void )
{
	int i;
	int j;
	uint32_t u32DataDirMask = 0;
	// enable the clock to port A
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// configure PORT E pins as GPIO
	
	for( i = 0; i < SENSOR_COUNT; i++ )
	{
		for( j = 0; j < LED_COUNT_PER_SENSOR; j++ )
		{
			PORTE->PCR[u8SensorLEDCfg[i][j]] &= ~PORT_PCR_MUX_MASK;
			PORTE->PCR[u8SensorLEDCfg[i][j]] |= PORT_PCR_MUX(1);
			u32DataDirMask |= MASK( u8SensorLEDCfg[i][j] );
		}
	}
	
//	PTE->PDDR |= u32DataDirMask;

//	PORTE->PCR[SENSOR_0_PORTE_LED0] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_0_PORTE_LED0] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_0_PORTE_LED1] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_0_PORTE_LED1] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_0_PORTE_LED2] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_0_PORTE_LED2] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_0_PORTE_LED3] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_0_PORTE_LED3] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_0_PORTE_LED4] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_0_PORTE_LED4] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_1_PORTE_LED0] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_1_PORTE_LED0] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_1_PORTE_LED1] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_1_PORTE_LED1] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_1_PORTE_LED2] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_1_PORTE_LED2] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_1_PORTE_LED3] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_1_PORTE_LED3] |= PORT_PCR_MUX(1);
//	
//	PORTE->PCR[SENSOR_1_PORTE_LED4] &= ~PORT_PCR_MUX_MASK;
//	PORTE->PCR[SENSOR_1_PORTE_LED4] |= PORT_PCR_MUX(1);
//	
//	// set pins connected to LEDs as outputs
//	PTE->PDDR |= MASK( SENSOR_0_PORTE_LED0 );
//	PTE->PDDR |= MASK( SENSOR_0_PORTE_LED1 );
//	PTE->PDDR |= MASK( SENSOR_0_PORTE_LED2 );
//	PTE->PDDR |= MASK( SENSOR_0_PORTE_LED3 );
//	PTE->PDDR |= MASK( SENSOR_0_PORTE_LED4 );

//	PTE->PDDR |= MASK( SENSOR_1_PORTE_LED0 );
//	PTE->PDDR |= MASK( SENSOR_1_PORTE_LED1 );
//	PTE->PDDR |= MASK( SENSOR_1_PORTE_LED2 );
//	PTE->PDDR |= MASK( SENSOR_1_PORTE_LED3 );
//	PTE->PDDR |= MASK( SENSOR_1_PORTE_LED4 );
}
