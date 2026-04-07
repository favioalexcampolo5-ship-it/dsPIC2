#ifndef MODBUS_APP_H
#define MODBUS_APP_H

#include <xc.h>
#include <stdint.h>
#include <libpic30.h>

#include "nanomodbus.h" // Para heredar los tipos de datos de la librería base

// ==========================================
// DEFINICIONES DE TAMAÑO DE MEMORIA
// ==========================================
// Estos límites deben coincidir con los de tu archivo .c
#define COILS_ADDR_MAX 10
#define REGS_ADDR_MAX 10

// ==========================================
// VARIABLES COMPARTIDAS (El puente con tu main)
// ==========================================
// El "extern" le avisa al main que estas variables existen físicamente 
// en tu archivo modbus_app.c, permitiéndole leerlas y escribirlas.
extern nmbs_bitfield server_coils;
extern uint16_t server_registers[REGS_ADDR_MAX + 1];

// ==========================================
// PROTOTIPOS DE LA API PÚBLICA
// ==========================================
void config_UART2(void);
void config_Timer2_1ms(void);
void Modbus_Init(void);
void Modbus_Tasks(void);

#endif	/* MODBUS_APP_H */