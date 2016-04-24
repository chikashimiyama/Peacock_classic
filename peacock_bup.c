

// Protocol
// 
// Packet A
// MSB	6	5	4	3	2	1	LSB  
// 1	S5	S4	S3	S2	S1	S0	D7
// 
//	Packet B
// MSB	6	5	4	3	2	1	LSB  
// 0	D6  D5  D4  D3  D2  D1  D0   
//
//

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/sfr_defs.h>:
#include <inttypes.h>

#define SPIMODE 0

#define FOSC 20000000                    // 20MHz
#define BAUD 38400
#define MYUBRR FOSC/16/BAUD-1    
#define TICK (1<<USIWM0)|(0<<USICS0)|(1<<USITC)
#define TOCK (1<<USIWM0)|(0<<USICS0)|(1<<USITC)|(1<<USICLK)


/* USI port and pin definitions.
 */
#define USI_OUT_REG	PORTB	//!< USI port output register.
#define USI_DIR_REG	DDRB		//!< USI port direction register.
#define USI_CLOCK_PIN	PORTB7	//!< USI clock I/O pin.
#define USI_DATAIN_PIN	PORTB5	//!< USI data input pin.
#define USI_DATAOUT_PIN	PORTB6	//!< USI data output pin.


/*  Speed configuration:
 *  Bits per second = CPUSPEED / PRESCALER / (COMPAREVALUE+1) / 2.
 *  Maximum = CPUSPEED / 64.
 */
#define TC0_PRESCALER_VALUE 256	//!< Must be 1, 8, 64, 256 or 1024.
#define TC0_COMPARE_VALUE   1	//!< Must be 0 to 255. Minimum 31 with prescaler CLK/1.


/*  Prescaler value converted to bit settings.
 */
#if TC0_PRESCALER_VALUE == 1
#define TC0_PS_SETTING (1<<CS00)
#elif TC0_PRESCALER_VALUE == 8
#define TC0_PS_SETTING (1<<CS01)
#elif TC0_PRESCALER_VALUE == 64
#define TC0_PS_SETTING (1<<CS01)|(1<<CS00)
#elif TC0_PRESCALER_VALUE == 256
#define TC0_PS_SETTING (1<<CS02)
#elif TC0_PRESCALER_VALUE == 1024
#define TC0_PS_SETTING (1<<CS02)|(1<<CS00)
#else
#error Invalid T/C0 prescaler setting.
#endif


// Received data from ADC
unsigned char storedUSIDR;

// Idle or convert
unsigned char status;

struct usidriverStatus_t {
	unsigned char masterMode : 1;       //!< True if in master mode.
	unsigned char transferComplete : 1; //!< True when transfer completed.
	unsigned char writeCollision : 1;   //!< True if put attempted during transfer.
};

volatile struct usidriverStatus_t spiX_status; //!< The driver status bits.


// Prototypes
void sio_init(unsigned int baud);
void blink(void);
char SPI_Transmit(unsigned char cData);
unsigned char checkStatus(void);
char spiX_put( unsigned char val );
unsigned char spiX_get(void);
void spiX_wait(void);

// Interrupts

ISR(USART_RX_vect)
{
	//if(bit_is_clear,UCSRA,FE)
	status = UDR;	
}

ISR(TIMER0_COMPA_vect)
{
	USICR |= (1<<USITC);	// Toggle clock output pin.
}

ISR(USI_OVERFLOW_vect)
{
	// Master must now disable the compare match interrupt
	// to prevent more USI counter clocks.
	if( spiX_status.masterMode == 1 ) {
		TIMSK &= ~(1<<OCIE0A);
	}
	
	// Update flags and clear USI counter
	USISR = (1<<USIOIF);
	spiX_status.transferComplete = 1;
	
	// Copy USIDR to buffer to prevent overwrite on next transfer.
	storedUSIDR = USIDR;
}

// Inits

void sio_init(unsigned int baud)
{
	UBRRH = (unsigned char)(baud>>8);   
	UBRRL = (unsigned char)baud; 
	UCSRB = (1<<RXEN)|(1<<TXEN) | (1<<RXCIE);
	UCSRC = (1<<USBS)|(3<<UCSZ0);		
}

void spiX_initmaster( char spi_mode )
{
	USI_OUT_REG |= (1<<USI_DATAIN_PIN);                       // Pull-ups.


	// Configure USI to 3-wire master mode with overflow interrupt.
	//USI setting
	USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (spi_mode<<USICS0) | (1<<USICLK);
	//6		USOIE ... Overflow Interupt Enable -> 1
	//5-4	USIWM ... Wire mode -> 01 (Three wire)
	//3-1	USICS ... Clock Source Select 101 -> Software Clock Strobe (USITC)
	//0		USITC ... Toggle Clock Port Pin
	//0b01011110
	
	// Enable 'Clear Timer on Compare match' and init prescaler.
	TCCR0A = (1<<WGM01) | TC0_PS_SETTING;
	
	// Init Output Compare Register.
	OCR0A = TC0_COMPARE_VALUE;
	
	// Init driver status register.
	spiX_status.masterMode       = 1;
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision   = 0;
	
	storedUSIDR = 0;
}


// Main

int main (void) {
	unsigned char adc, input, data, msg, count, tmp;

	DDRB = 0b11011111; // PORTB5 = Digital IN
	DDRD = 0b01111110; // PORTD0 = USART IN
	
	sio_init(MYUBRR);//USART init
	spiX_initmaster(SPIMODE);	// Init SPI driver as master.
	sei(); // enable interrupt

	
	status = 0;
	for(;;)
	{
		if (status == 0)
		{
			blink();			
		}
		else
		{
			for(adc = 0; adc <5; adc++)
			{	
				for(input = 0; input <7; input ++)
				{
					msg = 0b00011000 + input; // 8 + 16 + channel
					//msg = msg << 3; is it necessary?
					
					spiX_put( msg );	// Send temp value to SPI and increment,
					spiX_wait();		// wait for transmission to finish
					/*
					data = spiX_get();	// and finally put result on PORTB.	
					spiX_put( msg );	// Send temp value to SPI and increment,
					spiX_wait();		// wait for transmission to finish
					data = spiX_get();	// and finally put result on PORTB.	
					*/
					//sending data
					
					//making first packet//			
					while ( !(UCSRA & (1<<UDRE)) )
					count = adc *7 + input; // sensor ID
					tmp = count << 1;
					tmp += 128; // Ad ID
					tmp += data >> 7; // if more than 128
					{} //wait

					UDR = tmp;
					while ( !(UCSRA & (1<<UDRE)) )
					{} //wait
					//making second packet
					UDR = data & 0b01111111;
				}

			}
		}
	}
	return 0;
}





void blink(void)
{
	long int i,x;
	
	PORTD = 0x00;
	for (i = 0; i< 30000; i++)
	{
		i = i;
	}
	PORTD = 0xff;
	for (i = 0;i < 30000; i++)
	{
		i = i;
	}
}

/*! \brief  Put one byte on bus.
 *
 *  Use this function like you would write to the SPDR register in the native SPI module.
 *  Calling this function in master mode starts a transfer, while in slave mode, a
 *  byte will be prepared for the next transfer initiated by the master device.
 *  If a transfer is in progress, this function will set the write collision flag
 *  and return without altering the data registers.
 *
 *  \returns  0 if a write collision occurred, 1 otherwise.
 */

char spiX_put( unsigned char val )
{
	// Check if transmission in progress,
	// i.e. USI counter unequal to zero.
	if( (USISR & 0x0F) != 0 ) {
		// Indicate write collision and return.
		spiX_status.writeCollision = 1;
		return 1;
	}
	
	// Reinit flags.
	spiX_status.transferComplete = 0;
	spiX_status.writeCollision = 0;
	
	// Put data in USI data register.
	USIDR = val;
	
	// Master should now enable compare match interrupts.
	if( spiX_status.masterMode == 1 ) {
		TIFR |= (1<<OCF0A);   // Clear compare match flag.
		TIMSK |= (1<<OCIE0A); // Enable compare match interrupt.
	}
	
	if( spiX_status.writeCollision == 0 ) return 1;
	return 0;
}



/*! \brief  Get one byte from bus.
 *
 *  This function only returns the previous stored USIDR value.
 *  The transfer complete flag is not checked. Use this function
 *  like you would read from the SPDR register in the native SPI module.
 */
unsigned char spiX_get()
{
	return storedUSIDR;
}



/*! \brief  Wait for transfer to complete.
 *
 *  This function waits until the transfer complete flag is set.
 *  Use this function like you would wait for the native SPI interrupt flag.
 */
void spiX_wait()
{
	do {} while( spiX_status.transferComplete == 0 );
}
