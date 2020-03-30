// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdlib.h>
#include "twi_master.h"
#include "uart_functions.h"
#include "si4734.h"

extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;

volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

uint16_t current_fm_freq = 9910;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

//Used in debug mode for UART1
char uart1_tx_buf[40];      //holds string to send to crt
char uart1_rx_buf[40];      //holds string that recieves data from uart

//holds data to be sent to the segments. logic zero turns segment on 
uint8_t segment_data[5];
//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[11] = {0x03,0x9F,0x25,0x0D,0x99,0x49,0x41,0x1F,0x01,0x09,0xFF}; 


ISR(INT7_vect){STC_interrupt = TRUE;}

//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
uint8_t chk_buttons(uint8_t button) {

  static uint16_t state[8] ={0x00}; //holds present state
  state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
  if (state[button] == 0xF000) return 1;
  return 0;

}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
uint8_t ones;  
uint8_t tens;  
uint8_t hundreds;  
uint8_t thousands;  //determine how many digits there are 
ones = sum%10;
tens = (sum/10)%10;
hundreds = (sum/100)%10;
thousands = (sum/1000)%10;
segment_data[0] = dec_to_7seg[ones];
segment_data[1] = dec_to_7seg[tens];
segment_data[2] = dec_to_7seg[10];
segment_data[3] = dec_to_7seg[hundreds];
segment_data[4] = dec_to_7seg[thousands];
if(sum<10){           //blancking leading 0s
  segment_data[1] = 0xFF;
  segment_data[3] = 0xFF;
  segment_data[4] = 0xFF;}
else if(sum<100){
  segment_data[3] = 0xFF;
  segment_data[4] = 0xFF;}
else if(sum<1000){
  segment_data[4] = 0xFF;}

}

void init_ext()
 {
   EICRB |= (1<<ISC71) | (1<<ISC70);
   TIMSK |= (1<<INT7);
 }
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
init_twi();
//init_ext();
        //Setup audio output (max)
	DDRE  |= 0x08;
  TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);//initialize timer3 for vol
  TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS30);   //control
  TCCR3C = 0x00;
  OCR3A = 0x7000;
  ICR3 = 0xF000;

        //PORTE |= 0x08;
	//DDRE  |= 0x08;

	DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
	PORTE |= 0x04; //radio reset is on at powerup (active high)

        EICRB |= (1<<ISC71) | (1<<ISC70);
	EIMSK |= (1<<INT7);

	//hardware reset of Si4734
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);     //hold for 200us, 100us by spec         
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);      //5us required because of my slow I2C translators I suspect
	//Si code in "low" has 30us delay...no explaination
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

        sei();
//        init_ext();

	fm_pwr_up(); //powerup the radio as appropriate
	current_fm_freq = 9910; //arg2, arg3: 99.9Mhz, 200khz steps
	fm_tune_freq(); //tune radio to frequency in current_fm_freq

DDRB = 0xF0; //set portb 4-7 output
uint8_t disp[5] = {0x00,0x10,0x20,0x30,0x40};//slection array
int16_t sum = 0;
uint8_t a = 0;
uint8_t b = 0;
while(1){ 
	DDRA = 0x00;      // set porta to input
        PORTA = 0xFF;     // set porta to pull ups
	asm("nop");
	asm("nop");
	PORTB = 0x70;     // enable tri state buffer 
/*for(a=0;a<8;a++){         //loop delay for debounce and increment the count
	if(chk_buttons(0)){
		sum = sum + 1;
               _delay_ms(2);
	        }
       if(chk_buttons(1)){
                sum = sum + 2;
                _delay_ms(2);
                }
       if(chk_buttons(2)){
                sum = sum + 4;
                _delay_ms(2);
                }
       if(chk_buttons(3)){
                sum = sum + 8;
                _delay_ms(2);
                }
       if(chk_buttons(4)){
                sum = sum + 16;
                _delay_ms(2);
                }
        if(chk_buttons(5)){
                sum = sum + 32;
                _delay_ms(2);
                }
        if(chk_buttons(6)){
                sum = sum + 64;
                _delay_ms(2);
                }
        if(chk_buttons(7)){
                sum = sum + 128;
                _delay_ms(2);
                }

}*/
	if (sum>1023){    //boubd count to 0-1023
    		sum =1;
  	        }       
	segsum(current_fm_freq);//break up the disp_value to 4, BCD digits in the array: call (segsum)

	PORTB = 0x00;// disable tristate buffer
	DDRA = 0xFF;// set porta to output
	asm("nop");
	asm("nop");
	for(b=0;b<5;b++){             
		PORTB = disp[b];         //selection logic
		PORTA = segment_data[b]; //sending data to port a
               _delay_us(100);                 
         
                }//for
  }//while
}//main

