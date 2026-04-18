/*
 * File:   MAIN.c
 * Author: alex
 *
 * Created on April 16, 2026, 7:44 PM
 */


#include "xc.h"
#include "config.h"
#include "reloj.h"
#include <libpic30.h>

#define E1 TRISDbits.TRISD0
#define E2 TRISDbits.TRISD1
#define E3 TRISDbits.TRISD2

int main(void) {
    uint8_t sec = 0;
    uint8_t i = 0;
    uint16_t subida = 0X0080;
    uint8_t a = 0;
    ADPCFG = 0XFFFF;
    E1 = 1;
    E2 = 1;
    E3 = 1;
    TRISB = 0X0000;
    
    for(;;){
        if(PORTDbits.RD0 == 1){
            __delay_ms(10);
            if(PORTDbits.RD0 == 1){
                sec = 1;
            }
        }
        
        else if(PORTDbits.RD1 == 1){
            __delay_ms(10);
            if(PORTDbits.RD1 == 1){
                sec = 2;
            }
        }
        
        else if(PORTDbits.RD2 == 1){
            __delay_ms(10);
            if(PORTDbits.RD2 == 1){
                sec = 3;
            }
        }
        
        switch(sec){
            case 1:
                for(i = 0; i < 5; i++){
                    LATB = 0X0055;
                    __delay_ms(700);
                    LATB = 0X00AA;
                    __delay_ms(700);
                }
                LATB = 0x0000; // Apagar LEDs al terminar
                sec = 0;       // Evitar que se repita solo
                break;
            
            case 2:
                for(i = 0; i < 5; i++){
                    LATB = 0X000F;
                    __delay_ms(700);
                    LATB = 0X00F0;
                    __delay_ms(700);
                }
                LATB = 0x0000; // Apagar LEDs
                sec = 0;       // Evitar repetición
                break;
                
            case 3:
                subida = 0x0080; // Asegurar el valor inicial antes de empezar
                for(i = 0; i < 5; i++){
   
                    for(a = 0; a < 7; a++){
                        LATB = subida;         // <-- AQUÍ envías el valor a los LEDs
                        subida = subida>>1;
                        __delay_ms(400);
                    }
                    
                    for(a = 0; a < 7; a++){
                        LATB = subida;         // <-- AQUÍ también
                        subida = subida<<1;
                        __delay_ms(400);
                    }
                }
                LATB = 0x0000; // Apagar LEDs
                sec = 0;       // Evitar repetición
                break;
        }
    }
}
