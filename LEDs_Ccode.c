//
//  main.c
//  SeniorDesignProject_LEDsIndicators
//
//  Created by Abdullah Almethen on 8/14/21.
//

#include <stdio.h>

volatile short LED_Key_Pressed = 0;
void LEDs_Indicator(){
    
    /*Need to add a pin for the button, set as pull resistor and the add it in the condition of the first while loop instead of LED_Key_Pressed.
     Also, the code is just a template and a rough draft. It should be made to be incorporated in an interrupt to serve the function */
    
    
    
    //Getting the charge of the batteries
    int Battery_Charge = 100;//Should be set to the output of the balancing IC (battery charge)
    
    
    //Setting all the pins for the LEDs to be high to act as outputs
    //Red LED is set to pin 0 of port B and the rest are green LEDs
    DDRB |= (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4);
    
    
    
    while(LED_Key_Pressed){
    
    //Checking the output of the balancing IC to check the charge and the state of the batteries
        while(Battery_Charge >= 0){
        
            if(Battery_Charge <= 20){
                //Turning the red LED ON
                PORTB |= (1<<0);
                
                //Turning off all other LEDs just to make sure they are OFF
                PORTB &= ~(1<<1) & ~(1<<2) & ~(1<<3) & ~(1<<4);
                
                //Add a delay for each time for key is pressed to allow more for the user ot check the LED
                //To enhace the used UX
                _delay_ms(2000);
                
                }
            else if(Battery_Charge > 20 && Battery_Charge <= 40){
                
                //Turning the first green LED ON
                PORTB |= (1<<1);
            
                //Turning off all other LEDs just to make sure they are OFF
                PORTB &= ~(1<<0) & ~(1<<2) & ~(1<<3) & ~(1<<4);
                _delay_ms(2000);
            }
            else if(Battery_Charge > 40 && Battery_Charge <= 60){
                
                //Turning the first and second green LEDs ON
                PORTB |= (1<<1) | (1<<2);
                //Turning off all other LEDs just to make sure they are OFF
                PORTB &= ~(1<<0) & ~(1<<3) & ~(1<<4);
                _delay_ms(2000);
            }
            else if(Battery_Charge > 60 && Battery_Charge <= 80){
                //Turning the first, second and third green LEDs ON
                PORTB |= (1<<1) | (1<<2) | (1<<3);
                //Turning off all other LEDs just to make sure they are OFF
                PORTB &= ~(1<<0) & ~(1<<4);
                _delay_ms(2000);
            }
            else{
                //Turning all the green LEDs ON and the red LED OFF
                PORTB |= (1<<1) | (1<<2)| (1<<3)| (1<<4);
                //Turning off all other LEDs just to make sure they are OFF
                PORTB &= ~(1<<0);
                _delay_ms(2000);
            }
    
        }
}
