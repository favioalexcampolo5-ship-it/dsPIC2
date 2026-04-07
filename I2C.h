
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.  

char I2C_Rx(void);
void I2C_Tx(char data);
void I2C_Nack(void);
void I2C_Ack(void);
void I2C_Stop(void);
void I2C_Start(void);

#endif	/* XC_HEADER_TEMPLATE_H */

