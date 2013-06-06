/*
 *  Isaac Patka
 *  CLUE Summer 2013
 *  Radiator Labs
 */

#define F_CPU 8000000UL
#include "avr/io.h"
#include "util/delay.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#define LED_PIN 0b100000
#define RESET_SWITCH 0b1000


/*              *
 *  Prototypes  *
 *              */

//Set up UART preferences
void UART_INIT();

//Send some data
void UART_Transmit(unsigned char data);

//Receive some data
unsigned char UART_Receive();

//Initialize the ports
void Init_Ports();

//Debugging led
void blink_led();

void Reset_avr();

void Init_timer();

/*          *
 *  Global  *
 *Variables *
 *          */
 
 int transmit_timer_count = 0;
 unsigned char received_byte_a = 0;
  unsigned char received_byte_b = 0;
   unsigned char received_byte_c = 0;
   
 



int main(void)
{

    
    UART_INIT();
    Init_Ports();
    MCUSR = 0;
    wdt_disable();
    

    
    unsigned char instruction_byte;
    unsigned char RESET = 255;
    unsigned char BLINK_3X = 254;
    
    
    while (1)
    {
    

        received_byte_a = UART_Receive();
        received_byte_b = UART_Receive();
        received_byte_c = UART_Receive();
   
    Init_timer(); 
    
    instruction_byte = UART_Receive();  //pause while there is nothing in the receive buffer and transmit every 5 seconds based on timer interrupts
    
    
    if (instruction_byte == RESET)
    {   blink_led();
        blink_led();
     }
     
    else if (instruction_byte == BLINK_3X)
    {
        blink_led();
        blink_led();
        blink_led();
    }
    
        cli();  //DISABLE INTERRUPTS
        
 
    }

    return 0;
}

void UART_INIT()
{
    unsigned int UBBR_value;
    unsigned int baud = 19200;
    
    UCSR0A &= ~(_BV(U2X0));
    
    UBBR_value =    25;// ( ( 8000000 ) / (16 * baud)) - 1;    //unsigned int so it will round down to 25 @ baud rate of 19200. According to the data sheet (page 192) the error is 0.2%
    UBRR0H = (unsigned char) (UBBR_value >> 8);
    UBRR0L = (unsigned char) UBBR_value;
    
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     //8-bit data length
    
    //No parity set at this time
    
    UCSR0C &= ~(1 << USBS0);  //Sets 1 stop bit
    
    UCSR0B = (1 << RXEN0)  | (1 << TXEN0);   //Enable the receiver and transmitter
    
    
}

void UART_Transmit(unsigned char data)
{
    //Pause until transmitter is ready
    while ( !(UCSR0A & (1 << UDRE0)) );
    
    //Transmit the data
    UDR0 = data;
    
}

unsigned char UART_Receive()
{
    //Pause until RXC is nonzero
    while ( !(UCSR0A & (1 << RXC0)) );
    
    //Get data
    return UDR0;
    
}


void Init_Ports()
{
    
 
    DDRC |= (1 << PC5) | (1 << PC4);
    DDRC &= ~(1 << PC3);
                    

}
   

void blink_led()
{
  PORTC |= LED_PIN;
  _delay_ms(125);
  PORTC &= ~(LED_PIN);
  _delay_ms(125);
}

void Init_timer()
{
    TCCR1B |= (1 << WGM12); //configure timer 1 for CTC mode
    
    TIMSK1 |= (1 << OCIE1A);  //Enable CTC interrupt
    
    sei();  //enable Global interrupts 
    
    OCR1A = 31249;          //Set CTC compare value to 1Hz at 1MHz AVR clock, with prescaler of 256
    
    TCCR1B |= (1 << CS12);  //Start timer @ Fcpu/256
}

ISR(TIMER1_COMPA_vect)  //Execute this upon interrupt
{
    transmit_timer_count++;
    if (transmit_timer_count >=5)
    {
        /*int i = 0;
        for (i = 0; i <= 5; i++)
        UART_Transmit(i);*/
        
        UART_Transmit(100 + received_byte_a);
        UART_Transmit(105 + received_byte_a);
        UART_Transmit(110 + received_byte_a);
        UART_Transmit(150 + received_byte_b + received_byte_c);
        UART_Transmit(160 + received_byte_b + received_byte_c);
        UART_Transmit(170 + received_byte_b + received_byte_c);
        
        
        blink_led();
    
        transmit_timer_count = 0;
    }
    
}

void Reset_avr()
{
    wdt_enable(WDTO_30MS);
    while(1);
      

}
