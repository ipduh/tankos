/*
 * led.h
 *
 * Created: 6/5/2011 17:47:27
 * Author: sv1eed
 */ 


#ifndef LED_H_
#define LED_H_

#if defined(__AVR_ATmega168__) || \
    defined(__AVR_ATmega168P__) || \
    defined(__AVR_ATmega328P__)
	#define ledsetup	DDRB |= (1<<5)	// arduino led on portb 5 (D13)
	#define ledon		PORTB |= (1<<5)
	#define ledoff		PORTB &= ~(1<<5)
	#define ledtoggle	PINB |= (1<<5)

#elif defined(__AVR_ATmega1284P__)	// my led on 1284p is usually on portb0
	#define ledsetup	DDRB |= (1<<0)
	#define ledon		PORTB |= (1<<0)
	#define ledoff		PORTB &= ~(1<<0)
	#define ledtoggle	PINB |= (1<<0)
#endif

#endif /* LED_H_ */
