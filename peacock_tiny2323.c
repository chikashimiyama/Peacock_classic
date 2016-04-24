
// Target chip ATTiny2323
//
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
#include <avr/wdt.h>

#define SPIMODE 0						// Positive edge
#define FOSC 20000000                   // 20MHz
#define BAUD 38400						// Serial Speed 
#define MYUBRR FOSC/16/BAUD-1			// UBRR speed

/*  Speed configuration:
 *  Bits per second = CPUSPEED / PRESCALER / (COMPAREVALUE+1) / 2.
 *  Maximum = CPUSPEED / 64.
 */
#define TC0_PRESCALER_VALUE 64	//!< Must be 1, 8, 64, 256 or 1024.
#define TC0_COMPARE_VALUE   1	//!< Must be 0 to 255. Minimum 31 with prescaler CLK/1.

// 20000  / 8 / (10+1) / 2 = 200.0 bps ... ADC 0838 max speed = 10 - 400 kHz (according to the data sheet)


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
unsigned char test;


// Idle or convert
volatile unsigned char status = 0;

struct usidriverStatus_t {
	unsigned char transferComplete : 1; //!< True when transfer completed.
	unsigned char writeCollision : 1;   //!< True if put attempted during transfer.
};

volatile struct usidriverStatus_t spi_status; //!< The driver status bits.

// Prototypes

void uart_init(unsigned int baud);
char spi_put( unsigned char val );
unsigned char spi_get(void);
void spi_wait(void);
void blink(unsigned long int cycle);



ISR(USART_RX_vect)
{
	if(bit_is_clear(UCSRA,FE)) // avoid framing error
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
	//Timer Interrupt Mask Register
	TIMSK &= ~(1<<OCIE0A);
	
	//USI Overflow Interrupt flag -> Cleared
	USISR = (1<<USIOIF);

	//status update
	spi_status.transferComplete = 1;
	
	// Copy USIDR to buffer to prevent overwrite on next transfer.
	storedUSIDR = USIDR;
	
}

/*------- Initializations -------*/
void uart_init(unsigned int baud)
{
	UBRRH = (unsigned char)(baud>>8);   
	UBRRL = (unsigned char)baud; 
	UCSRB = (1<<RXEN)|(1<<TXEN) | (1<<RXCIE);
	UCSRC = (1<<USBS)|(3<<UCSZ0);		
}

void spi_initmaster( char spi_mode )
{
	// Configure USI to 3-wire master mode with overflow interrupt.
	//USI setting
	USICR = (1<<USIOIE) | (1<<USIWM0) | (1<<USICS1) | (spi_mode<<USICS0) | (1<<USICLK);
	//6		USOIE ... Overflow Interupt Enable -> 1
	//5-4	USIWM ... Wire mode -> 01 (Three wire)
	//3-1	USICS ... Clock Source Select 101 -> Software Clock Strobe (USITC)
	//0		USITC ... Toggle Clock Port Pin
	//0b01011110
	
	// Enable 'CTC'
	//Timer Counter Control Register
	TCCR0A = (1<<WGM01);
	//scale timer
	TCCR0B = TC0_PS_SETTING;
	
	// Init Output Compare Register.
	//Output Compare Unit - if 1 raise interrupt
	OCR0A = TC0_COMPARE_VALUE;
	
	// Init driver status register.
	spi_status.transferComplete = 0;
	spi_status.writeCollision   = 0;
	
	storedUSIDR = 0;
}


/*------- Main -------*/
int main (void) {
	
	unsigned char adc, input, data, msg, count, fpack;

	MCUSR = 0x00; // reset reset flags
	wdt_disable(); // stop wdt during setup
	
	DDRB = 0b11011111; // PORTB5 = Digital IN
	DDRD = 0b01111110; // PORTD0 = USART IN
	PORTB = 0b00011111; // five chips select + data in
	
	// waking up demo
	blink(40000);			
	blink(30000);			
	blink(20000);			
	blink(10000);			

	//Inits
	uart_init(MYUBRR);//USART init
	spi_initmaster(SPIMODE);	// Init SPI driver as master.
	sei(); // enable interrupt
	
	
	wdt_enable(WDTO_500MS); // start watch dog 0.5 sec cycle

	for(;;)
	{
		wdt_reset();
		if (status == 0)
		{
			blink(5000);
		}
		else
		{
			PORTD |= (1<<PORTD5);// LED always on
			for(adc = 0; adc <5; adc++)
			{	
				for(input = 0; input <7; input++)
				{
					// NULL | NULL | NULL | Start | Single | Ch | Ch | Ch | Mux Setting 
					msg = 0b000110000 + (input << 1);
				
					PORTB &= ~(1 << adc); // start -> pull down
					
					//request
					spi_put( msg );	// Send temp value to SPI and increment,
					spi_wait();		// wait for transmission to finish
					data = spi_get();	// and finally put result on data.
					
					//receive
					spi_put( msg );	// Send temp value to SPI and increment,
					spi_wait();		// wait for transmission to finish
					data = spi_get();	// and finally put result on data.	

					PORTB |= (1 << adc); // end -> pullup

					//making first packet//			
					count = adc *7 + input; // sensor ID
					fpack = count << 1; // LSB is for data
					fpack |= 0b10000000; // add ID
					fpack += data >> 7; // if more than 128
					
					while ( !(UCSRA & (1<<UDRE)) )
					{} //wait
					UDR = fpack;
					
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


/* ------- SPI routines -------*/
char spi_put( unsigned char val )
{
	// Check if transmission in progress,
	// i.e. USI counter unequal to zero.
	if( (USISR & 0x0F) != 0 ) {
		// Indicate write collision and return.
		spi_status.writeCollision = 1;
		return 1;
	}
	// Reinit flags.
	spi_status.transferComplete = 0;
	spi_status.writeCollision = 0;
	
	// Put data in USI data register.
	USIDR = val;
	
	//Timer Counter Interrupt Flag Register
	TIFR |= (1<<OCF0A);   // Clear compare match flag.
	TIMSK |= (1<<OCIE0A); // Enable compare match interrupt.
	
	if( spi_status.writeCollision == 0 ) return 1;
	return 0;
}

unsigned char spi_get()
{
	return storedUSIDR;
}

void spi_wait()
{
	do {} while( spi_status.transferComplete == 0 );
}

/*------- LED -------*/
void blink(unsigned long int cycle)
{
	unsigned int i,x;
	PORTD &= ~(1<<PORTD5);
	for (i = 0; i< cycle; i++)
	{
		x = i * i;
	}
	PORTD |= (1<<PORTD5);
	for (i = 0;i < cycle; i++)
	{
		x = i * i; 
	}
}


