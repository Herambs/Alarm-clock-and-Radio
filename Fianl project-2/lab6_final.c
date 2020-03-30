#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "twi_master.h"
#include "uart_functions.h"

extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;

volatile enum radio_band current_radio_band = FM;

//global variables for fm
uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;
uint16_t fm_freq = 9910;
uint16_t current_fm_freq = 9910;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume;

//Used in debug mode for UART1
char uart1_tx_buf[40];      //holds string to send to crt
char uart1_rx_buf[40];      //holds string that recieves data from uart


uint8_t flag = 0;
volatile uint8_t  rcv_rdy;
char    lcd_string_array[16];  //holds a string to refresh the LCD
uint16_t adc_result;  //holds adc result
uint16_t sum = 130; //holds real time clock value
uint8_t seconds = 0; //cunts seconds for real time clock
uint8_t seconds1 = 0; //counts seconds for snooze 
uint8_t b = 0;
uint8_t state = 0;    //holds push button value
uint8_t counter_value=0; //variable for incrementing according to push button
int16_t total = 131; //variable for alarm
uint8_t data = 0x00; 
uint16_t alarm = 0;
char   rx_char;
extern uint8_t lm73_wr_buf[2];  
extern uint8_t lm73_rd_buf[2];
uint8_t k;

void adjust_time(){   // bounding cont for real time clock
if(sum == 60){
     sum = 100;}
   if(sum == 160){
     sum = 200;}
if(sum == 260){
     sum = 300;}
   if(sum == 360){
     sum = 400;}
if(sum == 460){
     sum = 500;}
   if(sum == 560){
     sum = 600;}
if(sum == 660){
     sum = 700;}
   if(sum == 760){
     sum = 800;}
if(sum == 860){
     sum = 900;}
   if(sum == 960){
     sum = 1000;}
if(sum == 1060){
     sum = 1100;}
   if(sum == 1160){
     sum = 1200;}
if(sum == 1260){
     sum = 1300;}
   if(sum == 1360){
     sum = 1400;}
if(sum == 1460){
     sum = 1500;}
   if(sum == 1560){
     sum = 1600;}
if(sum == 1660){
     sum = 1700;}
   if(sum == 1760){
     sum = 1800;}
if(sum == 1860){
     sum = 1900;}
   if(sum == 1960){
     sum = 2000;}
if(sum == 2060){
     sum = 2100;}
   if(sum == 2160){
     sum = 2200;}
if(sum == 2260){
     sum = 2300;}
   if(sum == 2360){
     sum = 0;}
}

uint8_t disp[5] = {0x00,0x10,0x20,0x30,0x40};//slection array


//holds data to be sent to the segments. logic zero turns segment on 
uint8_t segment_data[5];

uint8_t segment_data1[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[14] = {0x03,0x9F,0x25,0x0D,0x99,0x49,0x41,0x1F,0x01,0x09,0x20,0xFF,0x00,0xDB}; 


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


//******************************************************************//
                           //button process
//******************************************************************//

uint8_t button_process(void)
  {
     uint8_t i=1;
     PORTB |= 0x70;                           // ENABLE TRISTATE BUFFER

      for(i=1;i<9;i++)                       //counter for push buttons
       {                       
         chk_buttons(i-1);                       // call chk_cuttons

          if(chk_buttons(i-1) == 1)              // if its pushed
           {
             state ^= i;                         //state holds value of push buttons
           }

          if((state & 0x03) == 0) {sei();}           //if no button pressed then enable global interrupt
 
          if((state & 0x07) == 1) {counter_value = 1;} //if first button is pressed

          if((state & 0x07) == 2) 
           {
             counter_value = 1;
             seconds = 0;            //if second button is pressed
           }
         if((state & 0x07) == 4)     //mode for controlling volume 
           {
             volume_control();        
           }

            
       } //for

  PORTB |= 0x00;                                //disable tristate buffer
  SPDR = state;                                  // send data to bargraph
  while(bit_is_clear(SPSR,SPIF)){}            // wait for 8 cycles
  data = SPDR;                                 //receive  data form encoder
 }



//******************************************************************//
                         //encoder
//*****************************************************************//

void encoder(){

 static uint8_t a_past = 0x00; // holds last value
 static uint8_t b_past = 0x00; // holds last value for 

 DDRE = 0xFF;                  //set port E to output
 PORTE |= 0x20 ;               // inhibitor high
 asm("nop");
 asm("nop");
 PORTE &= ~(0x40);             //SH_LD/ low
 asm("nop");
 asm("nop");
 PORTE |= 0x40;               //SH/LD_n high
 PORTE &= ~(0x20);            //inhibitor low
 uint8_t dat = data&0x03;     // mask data
 uint8_t dat1 = data&0x0C;    // mask data1

if((state & 0x03) == 1){
	switch(a_past)        // inc & dec according to direction of encoder for encoder 1
	{
	case 0x00:
		{
			if(dat==0x01){}
	          	else if(dat==0x02){}
			a_past = dat;
                  	break;
		}

        case 0x01:
		{
		  if(dat==0x03){total = total + counter_value;
                                        _delay_ms(2);}           // inc count
                  else if(dat==0x00){total = total - counter_value;
                                           _delay_ms(2);}        // dec count
			a_past = dat;
                  	break;
		}
        case 0x03:
		{
			if(dat==0x02){}
                  	else if(dat==0x01){}
			a_past = dat;
                  	break;
		}
        case 0x02:
		{
			if(dat==0x00){}
                  	else if(dat==0x03){}
			a_past = dat;
                  	break;
		}
	
    }
 }
if((state & 0x03) == 2){
switch(b_past)                 //inc and dec count according to direction of encoder for second encoder
        {
        case 0x00:
                {
                        if(dat1==0x04){}
                        else if(dat1==0x08){}
                        b_past = dat1;
                        break;
                }

        case 0x04:
                {
                        if(dat1==0x0C){sum = sum + counter_value;}      // inc count
                        else if(dat1==0x00){sum = sum - counter_value;} // dec count
                        b_past = dat1;
                        break;
                }
        case 0x0C:
                {
                        if(dat1==0x08){}
                        else if(dat1==0x04){}
                        b_past = dat1;
                        break;
                }
        case 0x08:
                {
                        if(dat1==0x00){}
                        else if(dat1==0x0C){}
                        b_past = dat1;
                        break;
                }

    }
  segsum(sum);               //display increment setting of real time clock
}
  
}

void set_freq(){      //tune different frequencies using encoder

 static uint8_t a_past = 0x00; // holds last value
 static uint8_t b_past = 0x00; // holds last value for 

 DDRE = 0xFF;                  //set port E to output
 PORTE |= 0x20 ;               // inhibitor high
 asm("nop");
 asm("nop");
 PORTE &= ~(0x40);             //SH_LD/ low
 asm("nop");
 asm("nop");
 PORTE |= 0x40;               //SH/LD_n high
 PORTE &= ~(0x20);            //inhibitor low
 uint8_t dat = data&0x03;     // mask data
 uint8_t dat1 = data&0x0C;    // mask data1


	switch(a_past)        // inc & dec according to direction of encoder for encoder 1
	{
	case 0x00:
		{
			if(dat==0x01){}
	          	else if(dat==0x02){}
			a_past = dat;
                  	break;
		}

        case 0x01:
		{
		  if(dat==0x03){current_fm_freq = current_fm_freq + 20;
                                        _delay_ms(2);}           // inc count
                  else if(dat==0x00){current_fm_freq = current_fm_freq - 20;
                                           _delay_ms(2);}        // dec count
			a_past = dat;
                  	break;
		}
        case 0x03:
		{
			if(dat==0x02){}
                  	else if(dat==0x01){}
			a_past = dat;
                  	break;
		}
        case 0x02:
		{
			if(dat==0x00){}
                  	else if(dat==0x03){}
			a_past = dat;
                  	break;
		}
	
    }
if((state & 0x03) == 2){
switch(b_past)                 //inc and dec count according to direction of encoder for second encoder
        {
        case 0x00:
                {
                        if(dat1==0x04){}
                        else if(dat1==0x08){}
                        b_past = dat1;
                        break;
                }

        case 0x04:
                {
                        if(dat1==0x0C){sum = sum + counter_value;}      // inc count
                        else if(dat1==0x00){sum = sum - counter_value;} // dec count
                        b_past = dat1;
                        break;
                }
        case 0x0C:
                {
                        if(dat1==0x08){}
                        else if(dat1==0x04){}
                        b_past = dat1;
                        break;
                }
        case 0x08:
                {
                        if(dat1==0x00){}
                        else if(dat1==0x0C){}
                        b_past = dat1;
                        break;
                }

    }
  segsum(sum);               //display increment setting of real time clock
}
  
}


void volume_control(){  //volume control using encoder

 static uint8_t a_past = 0x00; // holds last value
 static uint8_t b_past = 0x00; // holds last value for 

 DDRE = 0xFF;             //set port E to output
 PORTE |= 0x20 ;          // inhibitor high
 asm("nop");
 asm("nop");
 PORTE &= ~(0x40);        //SH_LD/ low
 asm("nop");
 asm("nop");
 PORTE |= 0x40;           //SH/LD_n high
 PORTE &= ~(0x20);        //inhibitor low
 uint8_t dat = data&0x03;  // mask data
 uint8_t dat1 = data&0x0C; // mask data1


	switch(a_past)  // inc & dec according to direction of encoder for encoder 1
	{
	case 0x00:
		{
			if(dat==0x01){}
	          	else if(dat==0x02){}
			a_past = dat;
                  	break;
		}

        case 0x01:
		{
		  if(dat==0x03){OCR3A = OCR3A - 0x0200;
                                        _delay_ms(2);}// inc count
                  else if(dat==0x00){OCR3A = OCR3A + 0x0200;
                                           _delay_ms(2);}// dec count
			a_past = dat;
                  	break;
		}
        case 0x03:
		{
			if(dat==0x02){}
                  	else if(dat==0x01){}
			a_past = dat;
                  	break;
		}
        case 0x02:
		{
			if(dat==0x00){}
                  	else if(dat==0x03){}
			a_past = dat;
                  	break;
		}
	
    }
if((state & 0x03) == 2){
switch(b_past)  //inc dec count according to direction of encoder for second encoder
        {
        case 0x00:
                {
                        if(dat1==0x04){}
                        else if(dat1==0x08){}
                        b_past = dat1;
                        break;
                }

        case 0x04:
                {
                        if(dat1==0x0C){sum = sum + counter_value;} // inc count
                        else if(dat1==0x00){sum = sum - counter_value;} // dec count
                        b_past = dat1;
                        break;
                }
        case 0x0C:
                {
                        if(dat1==0x08){}
                        else if(dat1==0x04){}
                        b_past = dat1;
                        break;
                }
        case 0x08:
                {
                        if(dat1==0x00){}
                        else if(dat1==0x0C){}
                        b_past = dat1;
                        break;
                }

    }
  segsum(sum);       //display increment setting of real time clock
}
  
}



//*********************************************************************
                                //SPI initialize
//*********************************************************************
void spi_init()
{
        DDRF  |= 0x08;  //port F bit 3 is enable for LCD
        PORTF &= 0xF7;  //port F bit 3 is initially low
	DDRB = 0xF7;    // set ss_n, sck, mosi
	SPCR |= (1<<SPE) | (1<<MSTR);// master mode
	SPSR |= (1<<SPI2X);// double speed operation
}

//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(sum) {
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
if((seconds % 2) == 0){segment_data[2] = dec_to_7seg[10];}
else segment_data[2] = dec_to_7seg[11];
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



void segsum1(sum1) {
uint8_t ones1;
uint8_t tens1;
uint8_t hundreds1;
uint8_t thousands1;  //determine how many digits there are 
ones1 = sum1%10;
tens1 = (sum1/10)%10;
hundreds1 = (sum1/100)%10;
thousands1 = (sum1/1000)%10;
segment_data1[0] = dec_to_7seg[ones1];
segment_data1[1] = dec_to_7seg[tens1];
if((seconds % 2) == 0){segment_data[2] = dec_to_7seg[12];}
else segment_data[2] = dec_to_7seg[11];
segment_data1[3] = dec_to_7seg[hundreds1];
segment_data1[4] = dec_to_7seg[thousands1];
if(sum<10){           //blancking leading 0s
  segment_data1[1] = 0xFF;
  segment_data1[3] = 0xFF;
  segment_data1[4] = 0xFF;}
else if(sum1<100){
  segment_data1[3] = 0xFF;
  segment_data1[4] = 0xFF;}
else if(sum1<1000){
  segment_data1[4] = 0xFF;}

}

void segsum2(uint16_t sum) {
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

void segsum3(uint16_t sum) {
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
segment_data[2] = dec_to_7seg[13];
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


void tcnt0_init(void)   //initializing timer/counter0 for real time clock
 {
   ASSR |= (1<<AS0);

  //Enable coutner in normal mode with no prescaler
  TCCR0 = (0<<CS02) | (0<<CS01) | (1<<CS00);

  //Wait for all ascynch warning bits to clear
  while(bit_is_set(ASSR, TCN0UB));
  while(bit_is_set(ASSR, OCR0UB));
  while(bit_is_set(ASSR, TCR0UB));

  //Enable overflow interrupts for T/C 0
  TIMSK |= (1<<TOIE0);
 }


ISR(USART0_RX_vect)
 {
   rx_char = UDR0;              //get remote temperature

   if(state == 5)
      {
        char2lcd(rx_char);
      }
 } 


ISR(TIMER0_OVF_vect)           //ISR for real time clock and snooze
 {
   static uint8_t count = 0;         
   count ++;

     if((count % 128) == 0)    //complete 1 sec
       {
         seconds = seconds + 1;//increment second

            if(seconds == 60)
              {           
                sum = sum + 1; //increment sum after every min
                seconds = 0;
              }
       }

       if(state == 3)         //snooze mode
         {
           if((count % 128) == 0)
             { 
                seconds1 = seconds1 + 1;  //increment seconds1 every second 
             }
         }
 }


void bargraph(){
  PORTB |= 0x01; //rising edge to rgclk
  PORTB &= ~0x01;//falling edge to rgclk
}


/**********************************************************************************
 initializing timer/counter2 in different configuration for controlling brightness
**********************************************************************************/

tcnt2_init(){                                //initializing timer/counter2 in different  
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);//configurtion for controling
OCR2 = 0x00;                                 //brightness 
}

tcnt21_init(){
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);
OCR2 = 0x2B;
}

tcnt22_init(){
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);
OCR2 = 0x56;
}

tcnt23_init(){
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);
OCR2 = 0x81;
}
tcnt24_init(){
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);
OCR2 = 0xAC;
}

tcnt25_init(){
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);
OCR2 = 0xD7;
}

tcnt26_init(){
TCCR2 |= (1<<WGM20) | (1<<COM21) | (1<<CS20);
OCR2 = 0xE0;
}

/**************************************************************************************/
 // initializing timer/counter1 in pwm mode for annoying beep
/*************************************************************************************/

void tcnt1_init()              //initializing timer/counter1 in pwm mode
 {
   TCCR1A = 0x00;                  //for annoying beep
   TCCR1B = (1<<WGM12) | (1<<CS11);
   TCCR1C = 0x00;
   OCR1A = 0x0200;
   TIMSK |= (1<<OCIE1A);
 }
 

ISR(TIMER1_COMPA_vect)
 {
   if((sum == total) && (state & 0x03) ==0){    
   PORTD ^= 0x80;}                              // RING alarm
   if((sum == total) && (state & 0x03) == 3){}  //if its snooze mode then dont play alarm
                                                  
     if(seconds1 >= 10)                         //after 10secs snooze play alarm
       {
         PORTD ^= 0x80;
       }
if(sum != total){}                        

  }//ISR
   

//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
uint16_t lm73_temp;
DDRF |= 0x08;//lcd strobe display
DDRD = 0x81;//for twi and annoying beep
DDRE = 0xFF;//sh/ld, clh_inh output


TCCR3A |= (1<<COM3A1) | (1<<COM3A0) | (1<<WGM31);//initialize timer3 for vol
TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS30);   //control
TCCR3C = 0x00;
OCR3A = 0x9000;//duty cycle of 60%
ICR3 = 0xF000;


uart_init();  //initialize uart
spi_init();   //initalize spi
lcd_init();   //initialize lcd
init_twi();   //initialize twi 
tcnt0_init(); //initialize timer0
tcnt1_init(); //initialize timer1
sei();        //enable global interrupts

DDRE  |= 0x04; //Port E bit 2 is active high reset for radio 
PORTE |= 0x04; //radio reset is on at powerup (active high)

//EICRB |= (1<<ISC71) | (1<<ISC70);
//EIMSK |= (1<<INT7);

//hardware reset of Si4734
PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
PORTE |=  (1<<PE2); //hardware reset Si4734 
_delay_us(200);     //hold for 200us, 100us by spec         
PORTE &= ~(1<<PE2); //release reset 
_delay_us(30);      //5us required because of my slow I2C translators I suspect
//Si code in "low" has 30us delay...no explaination
DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt

//sei();        //enable global interrupts

fm_pwr_up(); //powerup the radio as appropriate
current_fm_freq = 9910; //arg2, arg3: 99.9Mhz, 200khz steps

//SETTING UP ADC
DDRF &= ~(_BV(DDF7));//make port F bit 7 is ADC input  
PORTF &= ~(_BV(PF7));////port F bit 7 pullups must be off
ADMUX |= (1<<REFS0)|(1<<MUX2)|(1<<MUX1)|(1<<MUX0);//single-ended, input PORTF bit 7, right adjusted, 10 bits
ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//ADC enabled, don't start yet, single shot mode 


//set LM73 mode for reading temperature by loading pointer register
lm73_wr_buf[0] = LM73_PTR_TEMP; //load lm73_wr_buf[0] with temperature pointer address
twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);     //start the TWI write process
clear_display();

while(1)
 {
   if(state == 5) //display remote temperature and shut down alarm
     {
       set_cursor(0,0);
       total = 129;
       segsum(sum);
       asm("nop");
       asm("nop");
     }
  twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
  lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
  lm73_temp = (lm73_temp << 8); //shift it into upper byte 
  lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 
  itoa(lm73_temp>>7,lcd_string_array, 10); //convert to string in array with itoa() from avr-libc 


   ADCSRA |= (1<<ADSC);//poke ADSC and start conversion
   while(bit_is_clear(ADCSRA, ADIF)); //spin while interrupt flag not set
   ADCSRA |= (1<<ADIF); //its done, clear flag by writing a one 
   adc_result = ADC;  //read the ADC output as 16 bits
/**************************************************************************
      adjusting brightness according to adc value
**************************************************************************/
   if((0x03D4 < adc_result) && (adc_result < 0x03FF))
     {             
      tcnt2_init();
     }                                                  
  if((0x03A9 < adc_result) && (adc_result < 0x03D4))
     {
      tcnt21_init();
     }                                                       
  if((0x0379 < adc_result) && ( adc_result < 0x03A9))
     {
      tcnt22_init();
     }                                         
  if((0x0353 < adc_result) && (adc_result< 0x0379))
     {
      tcnt23_init();
     }
  if((0x0328 < adc_result) && (adc_result< 0x0353))
     {
      tcnt24_init();
     }     
  if((0x02FD < adc_result) && (adc_result< 0x0328))
     {
      tcnt25_init();
     }                                      


button_process();//check pushbuttons 
encoder(); //performs encoder function
adjust_time(); //call adjust_time

  if((state & 0x03) == 1) //mode for setting alarm
    {
        segsum2(total);
    }

bargraph(); //call bargraph
DDRA = 0x00; //set porta to input
PORTA = 0xFF; //porta to pullups
asm("nop");
asm("nop");


  if(((state & 0x07) == 0) && (seconds1 != 10)) //normal clock mode
    {
        clear_display(); 
        string2lcd(lcd_string_array); //send the string to LCD (lcd_functions)
        segsum(sum);
        seconds1 = 0;
    }

       if((state & 0x07) == 3)                         //snooze mode
         {
           segsum(sum);
         }

       if((sum == total) && (state & 0x03) == 0)       //alarm mode
         {
           clear_display();
           string2lcd("ALARM");                         //display "alarm" on lcd
           segsum1(sum);
         }

        if((seconds1 >= 10) && ((state & 0x07) == 3))  //after snooze
         {
           segsum(sum);
         }

        if((state & 0x08) == 8)                        //mode for tuning frequency
         {
          segsum3(current_fm_freq/10);
          set_freq();
            if((state & 0xFF) == 0x0C)
              {
                state = 0;
              }
          _delay_us(100);
         }
        if((state & 0x07) == 6)                     //mode for tuning radio on current frequency
         {
          segsum(current_fm_freq);
          set_freq();
          fm_tune_freq();
          _delay_ms(1);
         // state = 0;
         }
        if((state & 0x07) == 7)                    //mode for shutting down radio
         {
           radio_pwr_dwn();
           fm_pwr_up();
           state = 0;
         }

DDRA = 0xFF;                                    // set porta to output
asm("nop");
asm("nop");
        for(b=0;b<5;b++)
           {                              
	     PORTB = disp[b];                        //selection logic
	     PORTA = segment_data[b];                //sending data to port a
              _delay_ms(1);                 
         
           }//for
  }//while
}//main



