/*
 * tankos.c
 *
 * Created: 4/12/2014 3:20:00 PM
 * Author: g0, http://ipduh.com/contact
 * tankos v4 
 * 
 *  
 * 
 * If using an ATmega328 Arduino, then set the following
   Moters digital 2 , 3 , 7 , 8   
   SR04   digital 5 , 6
   Servo  digital 9  
 *
 *
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include "uart0_stream.h"
#include "led.h"
#include <avr/interrupt.h>
#include <util/delay.h>

//sonic eyes servo 
#define FACTOR			2
#define CENTER			1500*FACTOR
#define USER_OFFSET		580
#define OFFSET  USER_OFFSET*FACTOR  
#define LEFT	CENTER + OFFSET
#define RIGHT   CENTER - OFFSET
#define SMALL_LEFT		CENTER + USER_OFFSET
#define SMALL_RIGHT		CENTER - USER_OFFSET

#define HEAD_FAR_LEFT	OCR1A=LEFT
#define HEAD_FAR_RIGHT	OCR1A=RIGHT
#define HEAD_CENTER		OCR1A=CENTER
#define HEAD_SMALL_LEFT OCR1A=SMALL_LEFT
#define HEAD_SMALL_RIGHT OCR1A=SMALL_RIGHT

#define SAFEDISTANCE	30
#define SONIC_MEASUREMENTS_DELAY		150 //ms

// compile time debug options:
#define PRINT_DEBUG	0	//1
#define PRINT_DEBUG2 1 
#define debug_printf(fmt, ...) do { if (PRINT_DEBUG) printf(" *** %s(): " fmt, __func__, __VA_ARGS__); } while (0)
#define debug_printf2(fmt, ...) do { if (PRINT_DEBUG2) printf(" *** %s(): " fmt, __func__, __VA_ARGS__); } while (0)
#define debug_puts(a) 	if (PRINT_DEBUG) puts(a)
#define SLOW_DOWN_DEBUG 0

// SR04 - sonic eyes setup
#define trig_pin_setup() DDRD |= (1<<5)  // set as output
#define trig_pin_high()  PORTD |= (1<<5)
#define trig_pin_low()  PORTD &= ~(1<<5)
#define echo_pin_is_high() (PIND & (1<<6)) // PIND & 0b01000000
#define SR04_N_AVERAGE 3

// MOTORS
// PD2 & PD3 ( arduino digital 2 , 3 ) 
#define motorR_setup DDRD |= (1<<2) | (1<<3)

//purple & green motor cables
#define motorR_forward PORTD &= ~(1<<3); PORTD |= (1<<2);
#define motorR_back  PORTD &= ~(1<<2); PORTD |= (1<<3);  
//#define motorR_stop	 PORTD &= ~(1<<2) & ~(1<<3)
#define motorR_stop  PORTD &= ~( (1<<2) | (1<<3) )

// PD4 & PD7 ( arduino digital 4 , 7 )
#define motorL_setup DDRD |= (1<<4) | (1<<7)

//blue & green motor cables
#define motorL_forward PORTD &= ~(1<<7); PORTD |= (1<<4);
#define motorL_back PORTD &= ~(1<<4); PORTD |= (1<<7);
#define motorL_stop PORTD &= ~( (1<<7) | (1<<4))


#define TURN_90_MS 1100
#define SMALL_TURN_MS TURN_90_MS/2
#define TURN_45_MS TURN_90_MS/2


//Buttons - Bump Swithces
#define CHECK_BUTTONS_PERIOD 150 //ms
//PB0 - Digital 8
#define BUTTON_MID_CENTER		0
#define BUTTON_MID_CENTER_PIN	PINB

volatile uint8_t check_buttons_flag;
volatile uint8_t t2_ovf_counter_for_check_buttons_flag;

enum { zero , goleft , goright , stayput};

void timer2_CTC_setup_1ms()
{
	// setup timer0 (8 bit), using Clear Timer on Compare match (CTC) mode
	// Target Timer Count = (MCU Frequency / Prescale) / Target Frequency - 1
	#define target_ms_2  1        // milliseconds
	#define target_freq_2  (1000 / target_ms_2)    // Hz
	#define _OCR2A_VAL  F_CPU / 64 / target_freq_2 - 1
	
	//OCR0A = _OCR0A_VAL; // 249 (auto einai to TOP tou timer se auto to mode)
	OCR2A = 249;
	
	//#g0
	TCCR2A =  (1<<WGM21); // CTC MODE , UPPER BOUND OCR2A
	TCCR2B = (1<<CS22) ; // prescale to clkT2S/64
	TIMSK2 = (1<<OCIE2A); //enable Timer/Counter2 Compare Match A interrupt	
}

volatile uint16_t t2_ovf_counter_motorR;
volatile uint16_t t2_ovf_counter_motorL;

volatile uint8_t sonic_measurement_flag;
volatile uint16_t t2_ovf_counter_for_sonic_measurement_flag;

volatile uint16_t t2_ovf_counter_for_head_small_right_flag;
volatile uint16_t t2_ovf_counter_for_head_small_left_flag;

volatile uint8_t head_small_left_flag;        // 8 * SONIC_MEASUREMENTS_DELAY
volatile uint8_t head_small_right_flag;		  // 16 * SONIC_MEASUREMENTS_DELAY	

volatile uint8_t left_flag_counter=1;

volatile uint8_t stop;

ISR(TIMER2_COMPA_vect)
{
	if(t2_ovf_counter_motorR > 0)
	    t2_ovf_counter_motorR--;
	else
		motorR_stop;
	
	if(t2_ovf_counter_motorL > 0 )
		t2_ovf_counter_motorL--;
	else
		motorL_stop;
		
	if(t2_ovf_counter_for_check_buttons_flag < CHECK_BUTTONS_PERIOD)
		t2_ovf_counter_for_check_buttons_flag++;
	else
	{
		check_buttons_flag = 1;
		t2_ovf_counter_for_check_buttons_flag = 0;	
	}
		
	if(t2_ovf_counter_for_sonic_measurement_flag < SONIC_MEASUREMENTS_DELAY)
		t2_ovf_counter_for_sonic_measurement_flag++;
	else
	{
		sonic_measurement_flag = 1;
		t2_ovf_counter_for_sonic_measurement_flag = 0;	
	}
	
	if(t2_ovf_counter_for_head_small_left_flag < (8*SONIC_MEASUREMENTS_DELAY) )
		t2_ovf_counter_for_head_small_left_flag++;	
	else
	{
		if(left_flag_counter==1)
		{
			head_small_left_flag=1;
			left_flag_counter++;
		}
		else if(left_flag_counter==2)
		{
			head_small_left_flag=0;
			left_flag_counter=1;
		}	
		
		t2_ovf_counter_for_head_small_left_flag=0;	
	}
	
	if(t2_ovf_counter_for_head_small_right_flag < (16*SONIC_MEASUREMENTS_DELAY))
		t2_ovf_counter_for_head_small_right_flag++;
	else
	{
		head_small_right_flag=1;
		t2_ovf_counter_for_head_small_right_flag=0;
	}
		
}

void motorR_go_forward_for( uint16_t msin )
{
	t2_ovf_counter_motorR = msin;
	motorR_forward;	
}

void motorL_go_forward_for( uint16_t msin )
{
	t2_ovf_counter_motorL = msin;
	motorL_forward;
}

void motorL_go_back_for( uint16_t msin )
{
	t2_ovf_counter_motorL = msin;
	motorL_back;
}

void motorR_go_back_for( uint16_t msin )
{
	t2_ovf_counter_motorR = msin;
	motorR_back;		
}

void fast_right(uint16_t ms)
{
	motorR_go_back_for(ms);
	motorL_go_forward_for(ms);
}

void fast_left(uint16_t ms)
{
	motorL_go_back_for(ms);
	motorR_go_forward_for(ms);
}

void fast_small_right()
{
	motorR_go_back_for(SMALL_TURN_MS);
	motorL_go_forward_for(SMALL_TURN_MS);
	debug_printf2("fast_small_right\n %d , %d",t2_ovf_counter_motorL,t2_ovf_counter_motorR);	
}

void fast_small_right_no_returns()
{
	motorR_stop;
	motorL_stop;
	motorR_go_back_for(SMALL_TURN_MS);
	motorL_go_forward_for(SMALL_TURN_MS);
	debug_printf2(" ML_c:%d --MR_c:%d\n",t2_ovf_counter_motorL,t2_ovf_counter_motorR);
	
}


void fast_small_left()
{
	motorL_go_back_for(SMALL_TURN_MS);
	motorR_go_forward_for(SMALL_TURN_MS);
}

void both_motors_forward_for(uint16_t msin)
{
		motorR_go_forward_for(msin);
		motorL_go_forward_for(msin);	
}

void both_motors_back_for(uint16_t msin)
{
		motorR_go_back_for(msin);
		motorL_go_back_for(msin);
}


static inline void timer1_setup()
{
	// temp test
	// timer tick (o) = 1us

	// wraia epeidh me F_CPU 8mhz kai prescale 8 exoume timer freq 1mhz, ara tick time 1us

	#define timer1_tick_time_us  1

	// to get 1ms pulse, we need OCR1A = 1000us / timer1_tick_time_us
	
	//OUTPUT COMPARE REGISTER 1 A
	//OCR1A = 1200 / timer1_tick_time_us; // 1200 us FOR 8MHz Clock
	//OCR1A = 2400; // FOR 16MHz Clock
	OCR1A = CENTER;
	//OC1A digital 9
	
	//OUTPUT COMPATE REGISTER 1 B
	OCR1B = 1000; // timer1_tick_time_us; // 1000 us 
	
	//ICR1 = 20000; // ~50Hz FOR 8MHz we need T = 20 ms
	ICR1 = 40000; // ~50Hz FOR 16MHz

	//Table 16-2. Compare Output Mode, Fast PWM
	//Set OC1A/OC1B on Compare Match, clear
	//OC1A/OC1B at BOTTOM (inverting mode)
	
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);

	//Table 16-4. Waveform Generation Mode Bit Description
	//MODE 14, FAST PWM , TOP: ICR1

	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);  //start timer with prescaler 8
	
	//Table 16-5. Clock Select Bit Description
	//CS12  CS11  CS10
	//0     1     0     clkI/O/8 (From prescaler)
}

void timer0_CTC_setup_1ms()
{
	// setup timer0 (8 bit), using Clear Timer on Compare match (CTC) mode
	// Target Timer Count = (MCU Frequency / Prescale) / Target Frequency - 1
	// i.e.
	#define target_ms  1        // milliseconds
	#define target_freq  (1000 / target_ms)    // Hz
	#define _OCR0A_VAL  F_CPU / 64 / target_freq - 1
	
	//OCR0A = _OCR0A_VAL; // 249 (auto einai to TOP tou timer se auto to mode)
	OCR0A = 249;
	
	TCCR0B = (1<<WGM01) | (1<<CS01) | (1<<CS00); // CTC mode, presc 64
	TIMSK0 = (1<<OCIE0A); // enable compare match interrupt
}

volatile uint16_t t0_ovf_counter;
ISR(TIMER0_COMPA_vect)
{
	t0_ovf_counter++;
}

// return the distance in cm or zero on failure (timeout)
// it uses timer0 in timer0_CTC_setup_1ms
uint16_t SR04_measure()
{
	uint16_t result = 0;

	// setup timer here or permanently on main
	// ...

	// send ping pulse ( to request a new measurement )
	trig_pin_high();
	_delay_us(10);
	trig_pin_low();
	
	// reset timer counter and overflow counter
	TCNT0 = 0;
	t0_ovf_counter = 0;
	
	while(t0_ovf_counter < 20)	// 20ms timeout
	{
		if (echo_pin_is_high()) // high pulse received on echo pin
		{
			TCNT0 = 0;
			t0_ovf_counter = 0;
			
			while(t0_ovf_counter < 20) // wait for echo low
			{
				if( ! echo_pin_is_high()) // end of high pulse, i.e. end of measurement
				{
					//1000us = 249 tick ,therefore  1 tick ~ 4 us
					//measurement_uS = TCNT0 * 4 + 1000 * (uint16_t)t0_overflow_counter;
					
					// calculate usecs
					// 					4,016064257028112 us per tick
					result = TCNT0 * 4 + t0_ovf_counter*1000UL;	// small error but faster (max error ~ 0.6mm)
					
					//range = high level time * velocity (340M/S) / 2;
					//L = measurement_uS * 170;
					
					//HC-SR04.pdf: uS / 58 = centimeters
					result /= 58;
					#ifdef mydebug
					printf("TCNT0:%d \t Overflow_Counter:%u \t Distance:%u \n", TCNT0, t0_ovf_counter, result);
					#endif
					return result;
				}
			}
			
		}
	}
	// timeout while waiting for the rising edge
	return 0;
}

//IT DOES NOT WORK
//see if an average of many good measurements ie 3 is better
uint16_t SR04_measure_average()
{
	uint16_t result = 0;
	// setup timer here or permanently on main
	// ...
	
	uint8_t good_measurements=0;
	uint16_t sum=0;
	
	for(uint8_t i = 0; i < SR04_N_AVERAGE; i++)
	{
	// send ping pulse ( to request a new measurement )
	trig_pin_high();
	_delay_us(10);
	trig_pin_low();
	
	// reset timer counter and overflow counter
	TCNT0 = 0;
	t0_ovf_counter = 0;
	
	while(t0_ovf_counter < 20)	// 20ms timeout
	{
		if (echo_pin_is_high()) // high pulse received on echo pin
		{
			TCNT0 = 0;
			t0_ovf_counter = 0;
			while(t0_ovf_counter < 20) // wait for echo low
			{
				if( ! echo_pin_is_high()) // end of high pulse, i.e. end of measurement
				{
					result = TCNT0 * 4 + t0_ovf_counter*1000UL;	// small error but faster (max error ~ 0.6mm)
					result /= 58;
					//#ifdef mydebug
					//printf("TCNT0:%d \t Overflow_Counter:%u \t Distance:%u \n", TCNT0, t0_ovf_counter, result);
					//#endif
					sum+=result;
					good_measurements++;
					debug_printf("TCNT0:%d \t Overflow_Counter:%u \t Distance:%u \t M#:%d\n", TCNT0, t0_ovf_counter, result , good_measurements);
				}
			}
		}
	}
	
	}
	
	if ( sum == 0)
		return sum;
	else
		return (sum / good_measurements);
}


#define OUT 1

void measure_distance_and_print(uint16_t times)
{
	for(uint16_t i=0; i<times; i++)
	{
		_delay_ms(150);
		uint16_t distance = SR04_measure();
		printf("%u cm\n", distance);
	}
	printf("\n");
}

//TODO
void wall_ahead_decide_direction_void()
{
	#define N_DISTANCE_MEASUREMENTS 4
	uint16_t distance = 0;
	uint16_t right_clearance =0;
	uint16_t left_clearance = 0;
	uint16_t distance_sum = 0;
	uint16_t good_measurements_counter = 0;
	uint8_t moved = 0;
	
	if(OUT)
	{
		
	OCR1A=RIGHT;
	debug_puts("I 'll Look to my right\n");
	
	//sonic_measurement_flag=0;

	for(int i=0; i<N_DISTANCE_MEASUREMENTS;i++)
	{
		//if( sonic_measurement_flag )
		//{
		//	sonic_measurement_flag=0;

			_delay_ms(SONIC_MEASUREMENTS_DELAY);

			debug_printf("Right Distance:%d",distance);
			distance=SR04_measure();
		
			if(distance > 0)
			{
				distance_sum+=distance;
				good_measurements_counter++;
			}
		//}
	}
	if(distance_sum > 0 && good_measurements_counter > 0)
		right_clearance = distance_sum / good_measurements_counter;
	debug_printf("Right Clearance: %d \t Good Measurements: %d / %d \t ",right_clearance,good_measurements_counter,N_DISTANCE_MEASUREMENTS);
	
	distance_sum=0;
	good_measurements_counter=0;
	
	OCR1A=LEFT;
	debug_puts("I 'll look to my left\n");
	
	for(int i=0; i<N_DISTANCE_MEASUREMENTS;i++)
	{
		//if( sonic_measurement_flag )
		//{
			//sonic_measurement_flag=0;
			_delay_ms(SONIC_MEASUREMENTS_DELAY);
			distance=SR04_measure();
			debug_printf("left distance: %d\n",distance);
		
			if(distance > 0)
			{
				distance_sum+=distance;
				good_measurements_counter++;
			}
		//}
	}
	if(distance_sum > 0 && good_measurements_counter > 0)
		left_clearance = distance_sum / good_measurements_counter;
	debug_printf("Left Clearance: %d \t Good Measurements: %d / %d \t ",left_clearance,good_measurements_counter,N_DISTANCE_MEASUREMENTS);
	
	//
	OCR1A=CENTER;
	debug_puts("center\n");
	debug_printf("Right_Clearance:  %d\t Left_Clearance: %d\n",right_clearance,left_clearance);
	
	if(left_clearance > right_clearance)
	{
		if(left_clearance > SAFEDISTANCE)
		{
			debug_printf2("I will go Left - R:%d - L:%d\n",right_clearance,left_clearance);
			fast_left(TURN_45_MS);
			moved=1;
			//fast_small_left();
			//return goleft;
		}
	}
	
	if(right_clearance > left_clearance)
	{
		if(right_clearance > SAFEDISTANCE)
		{
			debug_printf2("I will go right - R:%d - L:%d\n",right_clearance,left_clearance);
			fast_right(TURN_45_MS);
			moved=1;
			//fast_small_right_no_returns();
			//return goright;
		}
	}
	
	if(right_clearance <= SAFEDISTANCE && left_clearance <= SAFEDISTANCE)
	{
		debug_printf2("I will move backwards - R:%d - L:%d \n",right_clearance,left_clearance);
		both_motors_back_for(TURN_45_MS);
		moved=1;
	}
	
	
	if(moved==0)
		both_motors_back_for(TURN_45_MS);
	
	//return stayput;
   }
}



uint16_t wall_ahead_decide_direction()
{
	#define N_DISTANCE_MEASUREMENTS 4
	uint16_t distance = 0;
	uint16_t right_clearance =0;
	uint16_t left_clearance = 0;
	uint16_t distance_sum = 0;
	uint16_t good_measurements_counter = 0;
	
	
	OCR1A=RIGHT;
	debug_puts("I 'll Look to my right\n");
	
	for(int i=0; i<N_DISTANCE_MEASUREMENTS;i++)
	{
			_delay_ms(SONIC_MEASUREMENTS_DELAY);
			distance=SR04_measure();
		
			if(distance > 0)
			{
				distance_sum+=distance;
				good_measurements_counter++;
			}
		
	}
	
	if(distance_sum > 0)
		right_clearance = distance_sum / good_measurements_counter;
	
	debug_printf("Right Clearance: %d \t Good Measurements: %d / %d \t ",right_clearance,good_measurements_counter,N_DISTANCE_MEASUREMENTS);
	
	if(SLOW_DOWN_DEBUG)
		_delay_ms(2000);
	
	distance_sum=0;
	good_measurements_counter=0;
	
	OCR1A=LEFT;
	debug_puts("I 'll look to my left\n");
	
	for(int i=0; i<N_DISTANCE_MEASUREMENTS;i++)
	{
		_delay_ms(SONIC_MEASUREMENTS_DELAY);
		distance=SR04_measure();
		debug_printf("left distance: %d\n",distance);
		
		if(distance > 0)
		{
			distance_sum+=distance;
			good_measurements_counter++;
		}
	}
	if(distance_sum > 0)
		left_clearance = distance_sum / good_measurements_counter;
	
	debug_printf("Left Clearance: %d \t Good Measurements: %d / %d \t ",left_clearance,good_measurements_counter,N_DISTANCE_MEASUREMENTS);
	
	if(SLOW_DOWN_DEBUG)
		_delay_ms(2000);
	
	//
	OCR1A=CENTER;
	debug_puts("center\n");
	debug_printf("Right_Clearance:  %d\t Left_Clearance: %d\n",right_clearance,left_clearance);
	
	if(left_clearance > right_clearance)
	{
		if(left_clearance > SAFEDISTANCE)
		{
			debug_printf2("I will go Left- R:%d L:%d\n",right_clearance,left_clearance);
			fast_small_left();
			return goleft;
		}
	}
	
	if(right_clearance > left_clearance)
	{
		if(right_clearance > SAFEDISTANCE)
		{
			debug_printf2("I will go right - R:%d L:%d\n",right_clearance,left_clearance);
			fast_small_right();
			return goright;
		}
	}
	
	if(right_clearance <= SAFEDISTANCE && left_clearance <= SAFEDISTANCE)
	{
		printf("I will stay put\n");
		return stayput;
	}
	
	return stayput;
}

#define TEST_DELAY _delay_ms(3000)
void test()
{
	printf("Testing ...\n");
	
	printf("HEAD_FAR_RIGHT\n");
	HEAD_FAR_RIGHT;
	TEST_DELAY;
	
	printf("HEAD_SMALL_RIGHT\n");
	HEAD_SMALL_RIGHT;
	TEST_DELAY;
	
	printf("HEAD_CENTER\n");
	HEAD_CENTER;
	TEST_DELAY;
	
	printf("HEAD_SMALL_LEFT\n");
	HEAD_SMALL_LEFT;
	TEST_DELAY;
	
	printf("HEAD_FAR_LEFT\n");
	HEAD_FAR_LEFT;
	TEST_DELAY;
	
	printf("HEAD_CENTER\n");
	HEAD_CENTER;
	TEST_DELAY;
		
	printf("fast_right(45)\n");
	fast_right(TURN_45_MS);
	TEST_DELAY;
	
	printf("fast_small_left\n");
	fast_small_left();
	TEST_DELAY;
	
	
	
//	printf("Turn head far left\n");
//	HEAD_FAR_LEFT;
//	TEST_DELAY;
	
	printf("fast_left(45)\n");
	fast_left(TURN_45_MS);
	TEST_DELAY;
	
	printf("fast_small_right\n");
	fast_small_right();
	TEST_DELAY;

/*	printf("Turn head far right\n");
	HEAD_FAR_RIGHT;
	TEST_DELAY;
	printf("Fast 90 degree Right\n");
	//fast_small_right();
	fast_right(TURN_90_MS);
	TEST_DELAY;
	
	printf("Turn head far left\n");
	HEAD_FAR_LEFT;
	TEST_DELAY;
	printf("Fast 90 degree Left\n");
	fast_left(TURN_90_MS);
	TEST_DELAY;

	printf("Turn Head to the center\n");
	HEAD_CENTER;
	TEST_DELAY;
	
	printf("Turn Head to small left\n");
	HEAD_SMALL_LEFT;
	TEST_DELAY;
	
	printf("Turn Head to small right\n");
	HEAD_SMALL_RIGHT;
	TEST_DELAY;
	
	printf("look at the center\n");
	HEAD_CENTER;
	TEST_DELAY;

	printf("Go forward\n");
	both_motors_forward_for(400);
	TEST_DELAY;

	printf("Go Back\n");
	both_motors_back_for(400);
	TEST_DELAY;
*/
	_delay_ms(10000);
}


volatile uint16_t contact_count;
volatile uint16_t no_contact;

void move()
{
	uint16_t s_distance;
	
	if( check_buttons_flag )
	{
		//debug_printf2("yo %d \n",cnt_on);
		check_buttons_flag=0;
		
		if (BUTTON_MID_CENTER_PIN & (1<<BUTTON_MID_CENTER))
		{
			printf("contact=%d \t no_contact=%d\n",contact_count , no_contact);
			ledoff;
			no_contact++;
		}
		else
		{
			//contact
			ledon;
			contact_count++;
			motorR_stop;
			motorL_stop;
			both_motors_back_for(TURN_45_MS*2);
			wall_ahead_decide_direction_void();
		}
	}
	
	if( sonic_measurement_flag  )
		{
			sonic_measurement_flag= 0;
			
			if(head_small_left_flag)
			{
				head_small_left_flag = 0;
				debug_printf2("HEAD_SMALL_LEFT : hslf:%d\n",head_small_left_flag);
				HEAD_SMALL_LEFT;
			}
			else if(head_small_right_flag)
			{
				head_small_right_flag = 0;
				HEAD_SMALL_RIGHT;
			}
			else
				HEAD_CENTER;
			
		    s_distance=SR04_measure();
			//distance=SR04_measure_average();
			debug_printf("Clearance Ahead:%d\n",s_distance);

			if(s_distance < SAFEDISTANCE && s_distance > 0)
			{
				s_distance=SR04_measure();
				//if small take one more measurement
				if(s_distance < SAFEDISTANCE && s_distance > 0)
				{
					debug_printf("SM - Clearance Ahead:%d\n",s_distance);
					motorR_stop;
					motorL_stop;
					wall_ahead_decide_direction_void();
					//_delay_ms(SONIC_MEASUREMENTS_DELAY);
					_delay_ms(TURN_45_MS);
				}
			}
			else
			{ 
					both_motors_forward_for(SONIC_MEASUREMENTS_DELAY);
			}
		}
}


int main(void)
{
	uint16_t distance;
	
	serialinit();
	debug_puts("init tankos v4\n\n");
	printf("init tankos v8\n\n");
	
	ledsetup;
	//ledon;
	//_delay_ms(2000);
	ledoff;
	
	//pull up on PB0 - Digital8
	PORTB |= (1<<BUTTON_MID_CENTER);
	
	
	//8b timer for the hypersonic eyes
	timer0_CTC_setup_1ms();
	
	//8b timer for motors and time based interrupts
	timer2_CTC_setup_1ms();
	motorR_setup;
	motorL_setup;
	
	stop=0;
	sei();             // enable interrupts
	
	//hypersonic eyes
	trig_pin_setup();

	//servo setup
	//OC1A is used for the servo PWM
	DDRB |= (1<<1) | (1<<2); // OC1A and OC1B set as outputs
	timer1_setup();

	_delay_ms(2000);
	
	HEAD_CENTER;
	//test();

	while(1)
	{
		move();
		//HEAD_SMALL_LEFT;
		//move();
		//HEAD_CENTER;
		//move();
		//HEAD_SMALL_RIGHT;
		//move();
		//HEAD_CENTER;
		//move();
		
	}
}
