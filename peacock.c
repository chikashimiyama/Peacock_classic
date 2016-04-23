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

void blink(unsigned long int cycle);
/*------- Main -------*/
int main (void) {
	
	unsigned char adc, input, data, msg, count, fpack;

	DDRB = 0b11110111; // PORT4 = out | PORT3 = IN | 
	DDRC = 0b00111111; // for chip select
	DDRD = 0b11111110; // URART transfer
	
	// waking up demo
	blink(40000);			
	blink(30000);			
	blink(20000);			
	blink(10000);			

	//Inits
	//uart_init(MYUBRR);//USART init
	sei(); // enable interrupt
	while(1)
	{
		blink(20000);
	}
	
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

/*------- LED -------*/
void blink(unsigned long int cycle)
{
	unsigned int i,x;
	PORTB &= ~(1<<PORTB0);
	for (i = 0; i< cycle; i++)
	{
		x = i * i;
	}
	PORTB |= (1<<PORTB0);
	for (i = 0;i < cycle; i++)
	{
		x = i * i; 
	}
}