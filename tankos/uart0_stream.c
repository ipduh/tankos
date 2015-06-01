/* 

 Authors: sv1eed, mdm2 & g0 aka sv1pdm 
 based on ATmega sheets , the internetz, trial and lots of error

 SERIAL 

 gia error 0%, exoume: (greenglish for: For 0% error : )
 fastest for 8mhz xtal = 500000 / 1000000 (X2)

 fastest for 16mhz is 1000000 / 2000000 (with U2X	 (tested kai me ceramic kai phge ok )
 fastest for 16mhz is 1000000 / 2000000 (with U2X) (the above comment is greenglish for: it did OK, when tested with ceramic caps )

 fastest for 10mhz is 625000 / 1250000 (2X)
 fastest for 20mhz is 1250000 / 2500000 (2X)
 CP21xx chips go up to 1000000 only and even for this they need configuration (utility)

*/

#include "uart0_stream.h"
#define BaudRate 500000UL	// this will be X 2, using the U2X0 
#define MYUBRR ((F_CPU / 16 / BaudRate ) - 1) 


#if defined (__AVR_ATmega8__)
#define UDR0 UDR
#define UBRR0L UBRRL
#define UBRR0H UBRRH
#define UCSR0A UCSRA
#define UCSR0B UCSRB
#define UCSR0C UCSRC
#define UDRE0 UDRE
#define RXC0 RXC
#define UCSZ01 UCSZ1
#define UCSZ00 UCSZ0
#define RXEN0 RXEN
#define TXEN0 TXEN
#define RXCIE0 RXCIE
#define TIMSK0 TIMSK
#define U2X0 U2X
#endif

static int uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void serialinit() {
 	/*Set baud rate */ 
	UBRR0H = (unsigned char)(MYUBRR>>8); 
	UBRR0L = (unsigned char) MYUBRR; 
	
	// double speed: 2M !! works ok with the FTDI
	UCSR0A |= (1<<U2X0);
	
	/* Enable receiver and transmitter   */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); 
	/* Frame format: 8data, No parity, 1stop bit */
	#if defined (__AVR_ATmega8__)
	UCSR0C = (1<<URSEL) | (1 << UCSZ01) | (1 << UCSZ00); // H MALAKIAAAA !!! 8ELEI URSEL !!!!
	#else
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	#endif
	
	stdout = &mystdout;
}

static int uart_putchar(char c, FILE *stream) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

