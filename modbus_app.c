#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "RELOJ.h"
#include <libpic30.h>
#include "modbus_app.h" 
#include "nanomodbus.h"

// =========================================================================
// 1. DEFINICIONES Y VARIABLES GLOBALES DEL MODBUS
// =========================================================================
#define RS485_CTRL LATDbits.LATD1  // Pin de Control (DE/RE) del MAX485
#define RTU_SERVER_ADDRESS 1       // La dirección de tu nodo en la red Modbus
#define COILS_ADDR_MAX 10          // Cantidad máxima de Coils (Bits)
#define REGS_ADDR_MAX 10           // Cantidad máxima de Registros (16 bits)

nmbs_platform_conf platform_conf;
nmbs_callbacks callbacks;
nmbs_t nmbs;

nmbs_bitfield server_coils = {0};
uint16_t server_registers[REGS_ADDR_MAX + 1] = {0};

// Variable para el reloj del sistema (milisegundos)
volatile uint32_t system_ms = 0;

// =========================================================================
// 2. EL RELOJ DEL SISTEMA (Timer 2 a 1 milisegundo exacto)
// =========================================================================
void config_Timer2_1ms(void) {
    T2CON = 0;             // Apagar Timer 2
    T2CONbits.TCKPS = 0;   // Prescaler 1:1
    TMR2 = 0;              // Reiniciar cuenta
    PR2 = 29490;           // (1 ms * 29.49 MIPS)
    
    IFS0bits.T2IF = 0;     // Limpiar bandera
    IEC0bits.T2IE = 1;     // Habilitar interrupción
    IPC1bits.T2IP = 2;     // Prioridad 2 (Baja)
    
    T2CONbits.TON = 1;     // Encender reloj
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;
    system_ms++; 
}

uint32_t get_current_time_ms(void) {
    return system_ms;
}

// =========================================================================
// HARDWARE: CONFIGURACIÓN UART2 (RS485)
// =========================================================================
void config_UART2(void) {
    TRISDbits.TRISD1 = 0; // RD1 como Salida (Control RS485)
    RS485_CTRL = 0;       // Arrancamos en Modo ESCUCHA

    U2MODEbits.UARTEN = 0;  
    U2BRG = 191;          // 9600 baudios para 29.49 MIPS
    
    U2MODEbits.PDSEL = 0b00; // 8 bits, sin paridad
    U2MODEbits.STSEL = 0;    // 1 bit de parada
    //U2MODEbits.ALTIO = 0;    // Pines estándar RF4 y RF5

    // Aquí NO encendemos la interrupción de recepción del UART2 (U2RXIE), 
    // porque NanoModbus lee el buffer manualmente usando la función read_serial.
    
    U2MODEbits.UARTEN = 1;  // Encender UART2
    U2STAbits.UTXEN = 1;    // Encender Transmisor
}

// =========================================================================
// 3. LA CAPA FÍSICA (Comunicación RS485 - Bare Metal)
// =========================================================================
int32_t write_serial(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    int32_t bytes_written = 0;
    
    RS485_CTRL = 1; // MODO TRANSMISIÓN
    __delay_us(50); // ESPERA A QUE EL 4N35 SE ENCIENDA
    
    while (bytes_written < count) {//EVALUA EL LIMITE DE BYTES DEL MODBUS
        if (!U2STAbits.UTXBF) {//SI AUN NO ESTA LLENO EL BUFFER LLENA UNO MAS
            U2TXREG = buf[bytes_written++];
        }
    }
    
    while(U2STAbits.TRMT == 0); // Esperar fin físico
    RS485_CTRL = 0; // MODO ESCUCHA
    
    return bytes_written;
}

int32_t read_serial(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    int32_t bytes_read = 0;

    while (bytes_read < count) {
        uint32_t start_time = get_current_time_ms();
        bool byte_received = false;

        while (1) {
            if (U2STAbits.URXDA) {
                buf[bytes_read++] = U2RXREG;
                byte_received = true;
                break; 
            }
            
            if (U2STAbits.OERR) {
                U2STAbits.OERR = 0; // Limpiar Overrun Error
                return -1;
            }
            
            if (byte_timeout_ms > 0 && ((get_current_time_ms() - start_time) >= (uint32_t)byte_timeout_ms)) {
                break; // Timeout Modbus
            }
        }
        if (!byte_received) break;
    }
    return bytes_read; 
}

// =========================================================================
// 4. LA INTELIGENCIA MODBUS (Callbacks)
// =========================================================================
void onError() {
    // Si hay un error fatal del protocolo, cae aquí
    // Puedes encender un LED de error o reiniciar el módulo
}

nmbs_error handle_read_coils(uint16_t address, uint16_t quantity, nmbs_bitfield coils_out, uint8_t unit_id, void* arg) {
    if (address + quantity > COILS_ADDR_MAX + 1) return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    for (int i = 0; i < quantity; i++) {
        bool value = nmbs_bitfield_read(server_coils, address + i);
        nmbs_bitfield_write(coils_out, i, value);
    }
    return NMBS_ERROR_NONE;
}

nmbs_error handle_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id, void* arg) {
    if (address + quantity > COILS_ADDR_MAX + 1) return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    for (int i = 0; i < quantity; i++) {
        nmbs_bitfield_write(server_coils, address + i, nmbs_bitfield_read(coils, i));
    }
    return NMBS_ERROR_NONE;
}

nmbs_error handle_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void* arg) {
    if (address > COILS_ADDR_MAX + 1) return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    nmbs_bitfield_write(server_coils, address, value);
    return NMBS_ERROR_NONE;
}

nmbs_error handle_read_holding_registers(uint16_t address, uint16_t quantity, uint16_t* registers_out, uint8_t unit_id, void* arg) {
    if (address + quantity > REGS_ADDR_MAX + 1) return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    for (int i = 0; i < quantity; i++) {
        registers_out[i] = server_registers[address + i];
    }
    return NMBS_ERROR_NONE;
}

nmbs_error handle_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t* registers, uint8_t unit_id, void* arg) {
    if (address + quantity > REGS_ADDR_MAX + 1) return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    for (int i = 0; i < quantity; i++) {
        server_registers[address + i] = registers[i];
    }
    return NMBS_ERROR_NONE;
}

nmbs_error handle_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void* arg) {
    if (address > REGS_ADDR_MAX + 1) return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    server_registers[address] = value;
    return NMBS_ERROR_NONE;
}

// =========================================================================
// 5. INICIALIZACIÓN Y BUCLE PRINCIPAL (El motor del Modbus)
// =========================================================================
void Modbus_Init(void) {
    config_UART2(); // Inicia el hardware serial (la función que armamos antes)
    config_Timer2_1ms(); // Inicia el reloj
    
    nmbs_platform_conf_create(&platform_conf);
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = read_serial;
    platform_conf.write = write_serial;
    platform_conf.arg = NULL;

    nmbs_callbacks_create(&callbacks);
    callbacks.read_coils = handle_read_coils;
    callbacks.write_multiple_coils = handle_write_multiple_coils;
    callbacks.write_single_coil = handle_write_single_coil;
    callbacks.read_holding_registers = handle_read_holding_registers;
    callbacks.write_multiple_registers = handle_write_multiple_registers;
    callbacks.write_single_register = handle_write_single_register;

    nmbs_error err = nmbs_server_create(&nmbs, RTU_SERVER_ADDRESS, &platform_conf, &callbacks);
    if (err != NMBS_ERROR_NONE) {
        onError();
    }

    // Tiempos ultra estrictos de Modbus RTU
    nmbs_set_read_timeout(&nmbs, 10);
    nmbs_set_byte_timeout(&nmbs, 5);
}

void Modbus_Tasks(void) {
    nmbs_error err;
    err = nmbs_server_poll(&nmbs);
    if (err == NMBS_ERROR_TRANSPORT) {
        onError();
    }
}
