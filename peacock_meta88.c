/* Name: peacock_mega88.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */


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
#define BAUD 38400						// Serial Speed
#define MYUBRR F_CPU/16/BAUD-1			// UBRR speed

/*  Speed configuration:
 *  Bits per second = CPUSPEED / PRESCALER / (COMPAREVALUE+1) / 2.
 *  Maximum = CPUSPEED / 64.
 */
#define TC0_PRESCALER_VALUE 64	//!< Must be 1, 8, 64, 256 or 1024.
#define TC0_COMPARE_VALUE   1	//!< Must be 0 to 255. Minimum 31 with prescaler CLK/1.

// 20000  / 8 / (10+1) / 2 = 200.0 bps ... ADC 0838 max speed = 10 - 400 kHz (according to the data sheet)


#define SPI_MESSAGE_MASK 0b01100000
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
#include <util/delay.h>


// Idle or convert
volatile uint8_t status = 0;


void uart_init(uint8_t baud);
void spi_initmaster();
uint8_t spiReadWrite(uint8_t out);
void blink(int delay);

ISR(USART_RX_vect){
    status = UDR0;
}


/*------- Initializations -------*/
void uart_init(uint8_t baud){
    DDRD |= (1 << PORTD1); // URART transfer

    UBRR0H = (uint8_t)(baud>>8);
    UBRR0L = (uint8_t)baud;
    UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0); // receive, send, receive interrupt
    UCSR0C = (1<<USBS0) | (3<<UCSZ00); // stop bit = 1 , 8 bits

}

void spi_initmaster(){
    
    // MOSI ... out / SCK ... out / SS ...out(in order to be in Master mode)
    DDRB |= (1 << PORTB3) | (1 << PORTB5) | (1 << PORTB2);
    DDRC |= (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3) | (1 << PORTC4) | (1 << PORTC5);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
    PORTC |= (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3) | (1 << PORTC4) | (1 << PORTC5);
}

void led_init(){
    DDRB |= (1 << PORTB1);
}

uint8_t spiReadWrite(uint8_t out){
    SPDR = out;
    while (!(SPSR & (1 << SPIF))){};
    return SPDR;
}

void uart_send(uint8_t value){
    while ( !(UCSR0A & (1<<UDRE0))){}
        UDR0 = value;
}


/*------- Main -------*/
int main (void) {
    
    unsigned char adc, input, data , msg, count, fpack, pin;
    

    
    //Inits
    uart_init(MYUBRR);//USART init
    spi_initmaster();
    led_init();
    
    
    // waking up demo
    blink(300);
    blink(220);
    blink(190);
    blink(160);
    
    
    sei(); // enable interrupt
    
    for(;;){
        
        if (status == 0U){
        
            blink(80);

        }else{
            
            PORTB |= (1<<PORTB1);// LED always on

            _delay_ms(1);

            for(adc = 0; adc < 5; adc++){
                pin = adc+1;

                for(input = 0; input <7; input++){
                    PORTC &= ~(1 << pin); // start -> pull down
                    msg = 0b01100000 + (input << 2);
                    spiReadWrite(msg);
                    data = spiReadWrite(0);
                    PORTC |= (1 << pin); // end -> pullup

                    //making first packet//
                    count = adc * 7 + input; // sensor ID

                    fpack = count << 1; // LSB is for data
                    fpack |= 0b10000000; // add ID
                    fpack += data >> 7; // if more than 128
                    
                    uart_send(fpack);
                    uart_send(data & 127 );

                }
            }
        }
        
    }
    return 0; // never reaches
}

/*------- LED -------*/
void blink(int delay){
    
    PORTB &= ~(1<<PORTB1);
    _delay_ms(delay);
    
    PORTB |= (1<<PORTB1);
    _delay_ms(delay);

}