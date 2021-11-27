#include "MKL25Z4.h"                    // Device header

#define BUS_CLOCK ( 10526316 )

#define SENSOR_COUNT ( 2 )
#define LED_COUNT_PER_SENSOR ( 5 )

#define LOWEST_VALID_CAPACITANCE ( 150 )
#define HIGHEST_VALID_CAPACITANCE ( 2050 )

#define LOWEST_VALID_TEMPERATURE ( -30 )
#define HIGHEST_VALID_TEMPERATURE ( 120 )

#define MASK(x) (1UL << (x)) 

#define LED_BLINK_PERIOD	500

#define WATER_LEVEL_0 315 
#define WATER_LEVEL_1	400
#define WATER_LEVEL_2	520
#define WATER_LEVEL_3 640
#define WATER_LEVEL_4 780
#define WATER_LEVEL_5 1000
#define WATER_LEVEL_6 1015

static void vInitializeUART2( uint32_t baud_rate ) ;
static void vInitializeLEDs( void );
static uint8_t UART2_Receive_Poll( void );
static void vUpdateDisplay( uint8_t u8WhichSensor, uint16_t u16Cap );
static void vInitializeLPTMR( void );
void LPTMR0_IRQHandler( void );

typedef struct xSensorStatusType
{
	uint16_t u16Cap;
	int8_t s8Temp;
	int8_t discard;
} xSENSOR_STATUS_TYPE;

static xSENSOR_STATUS_TYPE xSensorStatus[SENSOR_COUNT];

// all LEDs on port C
static const uint8_t u8SensorLEDCfg [SENSOR_COUNT][LED_COUNT_PER_SENSOR] = 
{
	{  7,  0,  3,  4, 5 },
	{  6, 10, 11,  9, 8 }
};
	
int main( void )
{
	uint8_t u8Buf[8];
	uint8_t u8BytesRxd = 0;
	vInitializeLEDs();
	vInitializeUART2( 4800 );
	vInitializeLPTMR();
	
	uint16_t u16Cap;
	
	while( 1 )
	{	
		u8Buf[ u8BytesRxd ] = UART2_Receive_Poll();
		
		u8BytesRxd++;
		
		if( u8BytesRxd >= 8 )
		{
			// received the entire payload, now parse to figure out what kind of reading it is
			
			if( u8Buf[0] < SENSOR_COUNT )
			{
				// received capacitance reading
				u16Cap = (uint16_t)( ( ( (uint16_t)u8Buf[2] ) << 8U ) | (uint16_t)u8Buf[3] );
			
				if( u16Cap >= LOWEST_VALID_CAPACITANCE && u16Cap <= HIGHEST_VALID_CAPACITANCE )
				{
					xSensorStatus[u8Buf[0]].u16Cap = u16Cap;
					vUpdateDisplay( u8Buf[0], u16Cap );
				}
			}
			
			u8BytesRxd = 0;
			
		}
	}
}

void LPTMR0_IRQHandler(void){
	if(xSensorStatus[0].u16Cap < WATER_LEVEL_1 && xSensorStatus[0].u16Cap != 0){
		PTC->PSOR = ( MASK( u8SensorLEDCfg[0][1] ) | 
		MASK( u8SensorLEDCfg[0][2] ) | MASK( u8SensorLEDCfg[0][3] ) | MASK( u8SensorLEDCfg[0][4] ) );	
		PTC->PTOR = MASK( u8SensorLEDCfg[0][0] );		
	}

	if(xSensorStatus[1].u16Cap < WATER_LEVEL_1 && xSensorStatus[1].u16Cap != 0){
		PTC->PSOR = ( MASK( u8SensorLEDCfg[1][1] ) | 
		MASK( u8SensorLEDCfg[1][2] ) | MASK( u8SensorLEDCfg[1][3] ) | MASK( u8SensorLEDCfg[1][4] ) );
		PTC->PTOR = MASK( u8SensorLEDCfg[1][0] );
	}
	
	// clear the overflow flag
	LPTMR0_CSR |= LPTMR_CSR_TCF_MASK;
}

void vInitializeLPTMR(){
		SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;  // Make sure clock is enabled
    LPTMR0_CSR = 0;                     // Reset LPTMR settings         
    LPTMR0_CMR = LED_BLINK_PERIOD;             // Set compare value (in ms)

    // Use 1kHz LPO with no prescaler
    LPTMR0_PSR = LPTMR_PSR_PCS(1) | LPTMR_PSR_PBYP_MASK;

		LPTMR0_CSR = LPTMR_CSR_TIE_MASK | LPTMR_CSR_TCF_MASK;
		LPTMR0_CSR |= LPTMR_CSR_TEN_MASK;

		NVIC_SetPriority(LPTMR0_IRQn, 3);
		NVIC_ClearPendingIRQ(LPTMR0_IRQn);
		NVIC_EnableIRQ(LPTMR0_IRQn);
	
		__enable_irq();
}

static void vUpdateDisplay( uint8_t u8WhichSensor, uint16_t u16Cap )
{
	int i;
	uint32_t u32TurnOffLEDMask = 0;
	
	// first turn off all the LEDs for the sensor
	
	for( i = 0; i < LED_COUNT_PER_SENSOR; i++ )
	{ 
		// active low LEDs, so set high to turn them off
		u32TurnOffLEDMask |= ( MASK( u8SensorLEDCfg[u8WhichSensor][i] ) );
	}
	
	// LEDs for u8WhichSensor should now all be off
	
	if( u16Cap < WATER_LEVEL_0 )
	{
		// blink LED0
		
	}
	else if( ( u16Cap >= WATER_LEVEL_1 ) && ( u16Cap < WATER_LEVEL_2 ) )
	{
		// light LED 0
		// first clear all of the LEDs
		PTC->PSOR = u32TurnOffLEDMask;
		PTC->PCOR = MASK( u8SensorLEDCfg[u8WhichSensor][0] );
	}
	else if( ( u16Cap >= WATER_LEVEL_2 ) && ( u16Cap < WATER_LEVEL_3 ) )
	{
		// light LED 0 and 1
		PTC->PSOR = u32TurnOffLEDMask;
		PTC->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) );
	}
	else if( ( u16Cap >= WATER_LEVEL_3 ) && ( u16Cap < WATER_LEVEL_4 ) )
	{
		// light LED 0, 1 and 2
		PTC->PSOR = u32TurnOffLEDMask;
		PTC->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) | 
		MASK( u8SensorLEDCfg[u8WhichSensor][2] ) );
	}
	else if( ( u16Cap >= WATER_LEVEL_4 ) && ( u16Cap < WATER_LEVEL_5 ) )
	{
		// light LED 0, 1, 2, 3
		PTC->PSOR = u32TurnOffLEDMask;
		PTC->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) | 
		MASK( u8SensorLEDCfg[u8WhichSensor][2] ) | MASK( u8SensorLEDCfg[u8WhichSensor][3] ) );
	}
	else if( u16Cap >= WATER_LEVEL_5 )
	{
		// light LED 0, 1, 2, 3 and 4
		PTC->PSOR = u32TurnOffLEDMask;
		PTC->PCOR = ( MASK( u8SensorLEDCfg[u8WhichSensor][0] ) | MASK( u8SensorLEDCfg[u8WhichSensor][1] ) | 
		MASK( u8SensorLEDCfg[u8WhichSensor][2] ) | MASK( u8SensorLEDCfg[u8WhichSensor][3] ) | MASK( u8SensorLEDCfg[u8WhichSensor][4] ) );
	}
	else
	{
		// blink LED 0, 1, 2, 3, and 4
	}
}

static uint8_t UART2_Receive_Poll( void ) 
{
		// wait until receive data register is full
		while( !( UART2->S1 & UART_S1_RDRF_MASK ) );
		return UART2->D;
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
	
	// No parity, 8 bits, one stop bit, other settings;
	UART2->C1 = UART2->S2 = UART2->C3 = 0;
	
	// Enable transmitter and receiver
	UART2->C2 = ( UART_C2_TE_MASK | UART_C2_RE_MASK );
}


static void vInitializeLEDs( void )
{
	int i;
	int j;
	uint32_t u32DataDirMask = 0;
	// enable the clock to port C
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	
	uint8_t u8Val;
	
	// configure PORT E pins as GPIO
	
	for( i = 0; i < SENSOR_COUNT; i++ )
	{
		for( j = 0; j < LED_COUNT_PER_SENSOR; j++ )
		{
			// configure each LED pin to be a GPIO
			u8Val = u8SensorLEDCfg[i][j];
			PORTC->PCR[u8SensorLEDCfg[i][j]] &= ~PORT_PCR_MUX_MASK;
			PORTC->PCR[u8SensorLEDCfg[i][j]] |= PORT_PCR_MUX(1);
			
			// each LED will need to be configured as an output
			u32DataDirMask |= MASK( u8SensorLEDCfg[i][j] );
		}
	}
	
	PTC->PDDR |= u32DataDirMask;
	
	// start with all LEDs off
	PTC->PSOR = 0xFFFFFFFF;
}
