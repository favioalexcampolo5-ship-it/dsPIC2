/*
 * File:   main_principal.c
 * Author: alex
 *
 * Created on April 2, 2026, 10:28 PM
 * Descripción: Nodo esclavo Modbus RTU para monitoreo predictivo de motores DC.
 * Capta vibración y corriente, y se comunica con Python (Maestro) vía RS485.
 */

#include <stdint.h>      // Para tipos de datos de tamaño exacto (uint16_t, etc.)
#include <stdio.h>       
#include <stdbool.h>     // Para el manejo de variables booleanas (true/false)
#include "xc.h"          // Cabecera principal del compilador XC16 para dsPIC
#include "config.h"      // Fusibles de configuración del hardware (FOSC, WDT, etc.)
#include "RELOJ.h"       // Librería para las macros de retardo (__delay_ms)
#include <libpic30.h>    
#include "modbus_app.h"  // Cabecera que integra el motor Modbus, UART2 y Timer2

#define FFT_SAMPLES 256  // Tamaño exacto del bloque de datos requerido para la FFT

// Prototipos de las funciones de configuración y control
void config_timer1(void);
void config_adc();
uint16_t leer_ADC(uint8_t canal);
void Ajustar_Muestreo_Dinamico(float rpm);

// Estructura principal que centraliza toda la memoria, banderas y estado del sistema
typedef struct{
    volatile uint8_t error_parametros;     // Bandera de seguridad si llegan RPM fuera de rango
    volatile uint8_t sistema_configurado;  // Indica si el sistema ya salió del modo Standby
    volatile uint16_t corriente_final;     // Valor instantáneo listo para lectura vía Modbus
    volatile uint16_t vibracion_final;     // Valor instantáneo listo para lectura vía Modbus
    
    // Arreglos de captura en RAM (ocupan 512 bytes en total)
    uint16_t buffer_corriente[FFT_SAMPLES]; // El Timer guarda aquí las muestras de corriente
    uint16_t buffer_vibracion[FFT_SAMPLES]; // El Timer guarda aquí las muestras de vibración

    // Variables de control para el llenado de los arreglos
    volatile uint16_t buffer_index;         // Contador de muestras (de 0 a 255)
    volatile bool bandera_buffer_lleno;     // Se activa al completar la captura del bloque
    
    // Unión para empaquetar las alertas y optimizar el uso de RAM (1 solo byte transmitido)
    union{
        uint8_t byte_alertas; // Acceso al byte completo para el envío rápido por Modbus
        struct{
            unsigned sobrecorriente      : 1; // Bit 0: Motor atascado o sobrecarga mecánica
            unsigned vibracion_alta      : 1; // Bit 1: Falla estructural o desbalanceo
            unsigned sensor_desconectado : 1; // Bit 2: Caída a cero absoluto en la lectura del ADC
            unsigned timeout_scada       : 1; // Bit 3: Pérdida de comunicación con el maestro
            unsigned falla_fft           : 1; // Bit 4: Colisión entre el cálculo matemático y el muestreo
            unsigned voltaje_bajo        : 1; // Bit 5: Caída de tensión en la alimentación de la placa
            unsigned reserva1            : 1; // Bit 6: Pin libre para expansiones futuras
            unsigned reserva2            : 1; // Bit 7: Pin libre para expansiones futuras
        }bits; // Acceso bit a bit para disparar alarmas individuales desde la lógica
    }alertas;
}estado_sistema_t;

estado_sistema_t estado_actual; // Instancia global de la estructura en memoria RAM

int main(void) {
    while(OSCCONbits.LOCK != 1); // Esperar a que el reloj PLL interno se estabilice físicamente
    
    // Inicialización en cero de la memoria para evitar la lectura de basura electrónica al encender
    estado_actual.error_parametros = 0;
    estado_actual.sistema_configurado = 0;
    estado_actual.corriente_final = 0;
    estado_actual.vibracion_final = 0;
    estado_actual.buffer_index = 0;
    estado_actual.bandera_buffer_lleno = false;
    estado_actual.alertas.byte_alertas = 0;

    TRISBbits.TRISB9 = 0; // Pin del LED indicador configurado como salida
    TRISDbits.TRISD9 = 0; // Pin de testeo de frecuencia configurado como salida
    
    config_timer1(); // Levantar el hardware del Timer 1 (inicia apagado por seguridad)
    config_adc();    // Inicializar el convertidor analógico a digital
    
    Modbus_Init();   // Levantar la UART2, el RS485, el Timer 2 y el servidor Modbus
    
    // Variable para controlar el parpadeo del LED sin usar delays que bloqueen la CPU
    uint32_t tiempo_ultimo_parpadeo = 0; 
    
    for(;;){
        
        // 1. MOTOR DE RED: Debe ejecutarse permanentemente para atender las peticiones Modbus
        Modbus_Tasks(); 
        
        // 2. MÁQUINA DE ESTADOS PRINCIPAL
        if(estado_actual.sistema_configurado == 0) {
            // == ESTADO: STANDBY (Esperando recibir la configuración de RPM desde Python) ==
            
            // Lógica de parpadeo del LED cada 500ms basada en la marca de tiempo del Timer 2
            if(get_current_time_ms() - tiempo_ultimo_parpadeo >= 500) {
                LATBbits.LATB9 = !LATBbits.LATB9;               
                tiempo_ultimo_parpadeo = get_current_time_ms(); 
            }
            
            // Verificar si el maestro (Python) escribió las RPM en el Holding Register 0
            uint16_t rpm_recibidas = server_registers[0];
            if(rpm_recibidas >= 10 && rpm_recibidas <= 10000) {
                Ajustar_Muestreo_Dinamico((float)rpm_recibidas); // Calcular prescaler y encender muestreo
                LATBbits.LATB9 = 1;                              // Dejar el LED fijo para indicar el arranque
            }
        } 
        else {
            // == ESTADO: SISTEMA CORRIENDO (Muestreo dinámico y telemetría activos) ==
            
            // Mapeo de los datos del dsPIC hacia los registros Modbus para su lectura externa
            server_registers[1] = (uint16_t)estado_actual.alertas.byte_alertas; // Envío de alertas empaquetadas
            server_registers[2] = estado_actual.corriente_final;                // Telemetría de corriente
            server_registers[3] = estado_actual.vibracion_final;                // Telemetría de vibración
            
            // Verificar si el Timer 1 ya completó la captura del bloque de 256 muestras
            if (estado_actual.bandera_buffer_lleno == true) {
                // (Espacio reservado para el procesamiento de la Transformada de Fourier)
                
                estado_actual.buffer_index = 0;              // Reiniciar el apuntador del arreglo
                estado_actual.bandera_buffer_lleno = false;  // Habilitar al Timer 1 para la siguiente captura
            }
        }
    }
}

// Interrupción del Timer 1: Se dispara automáticamente a la frecuencia de muestreo calculada
void __attribute__((__interrupt__, no_auto_psv))_T1Interrupt(void){
    IFS0bits.T1IF = 0;                // Obligatorio: Limpiar bandera de hardware para evitar un ciclo infinito
    LATDbits.LATD9 = !LATDbits.LATD9; // Toggle de pin para verificación de la frecuencia de muestreo con osciloscopio
    
    // Lectura secuencial de los canales del ADC 
    uint16_t i_x1 = leer_ADC(0);
    uint16_t i_x11 = leer_ADC(1);
    uint16_t v_x1 = leer_ADC(4);
    uint16_t v_x11 = leer_ADC(5);
    
    uint16_t corriente_valida;
    uint16_t vibracion_valida;
    
    // Lógica de selección automática de ganancia (Hardware Auto-Ranging)
    if(i_x11 > 4050 || i_x11 < 25){     // Si la señal amplificada se satura o cae en ruido de fondo...
        corriente_valida = i_x1;        // ...rescatar y utilizar la señal original
    } else {
        corriente_valida = i_x11;       // ...si la señal es estable, utilizar la lectura amplificada
    }
    
    if(v_x11 > 4050 || v_x11 < 25){     // Aplicar la misma lógica para el sensor de vibración
        vibracion_valida = v_x1;
    } else {
        vibracion_valida = v_x11;
    }
    
    // Guardar los datos filtrados en las variables instantáneas para el polling de Modbus
    estado_actual.corriente_final = corriente_valida;
    estado_actual.vibracion_final = vibracion_valida;
    
    // Si el bloque de memoria aún tiene espacio disponible...
    if(estado_actual.bandera_buffer_lleno == false) {
        // ...almacenar los valores leídos en la posición actual del arreglo
        estado_actual.buffer_corriente[estado_actual.buffer_index] = corriente_valida;
        estado_actual.buffer_vibracion[estado_actual.buffer_index] = vibracion_valida;
        
        estado_actual.buffer_index++; // Avanzar el apuntador a la siguiente dirección
        
        // Comprobar si se alcanzó el límite de muestras requeridas para la FFT
        if(estado_actual.buffer_index >= FFT_SAMPLES) {
            estado_actual.bandera_buffer_lleno = true; // Notificar al bucle principal que el bloque está listo
        }
    }
}

// Función central de control: Ajusta la física del Timer basándose en las RPM ingresadas
void Ajustar_Muestreo_Dinamico(float rpm) {
    if (rpm < 10.0 || rpm > 10000.0) {               // Filtro de seguridad ante datos de red corruptos
        T1CONbits.TON = 0;                           // Detener el muestreo por seguridad
        estado_actual.error_parametros = 1;          // Activar bandera interna de error de configuración
        estado_actual.sistema_configurado = 0;       // Forzar el retorno de la máquina de estados a Standby                       
        return;                                      
    }
    
    estado_actual.error_parametros = 0;              // Limpiar bandera de error tras validar el dato
    float f_nominal = rpm / 60.0;                    // Conversión de RPM a frecuencia mecánica (Hz)
    float f_muestreo = f_nominal * 30.0;             // Frecuencia de muestreo ajustada (cumple criterio de Nyquist)
    
    if (f_muestreo > 5000.0) f_muestreo = 5000.0;    // Límite físico de 5kHz para evitar la saturación de la CPU
    
    T1CONbits.TON = 0;                               // Apagar el Timer 1 temporalmente para modificar sus registros
    TMR1 = 0;                                        // Limpiar cuentas residuales en el registro del Timer

    // Cálculo matemático de los pulsos necesarios basado en el reloj del sistema (29.49 MIPS)
    unsigned long temp_pr1 = ((29491200UL / (unsigned long)f_muestreo) - 1);

    // Lógica para asignar el Prescaler correcto sin causar desbordamiento en el registro de 16 bits (PR1)
    if (temp_pr1 <= 65535) {
        T1CONbits.TCKPS = 0b00;         // Prescaler 1:1 (Máxima resolución)
        PR1 = (uint16_t)temp_pr1;       
    } 
    else if ((temp_pr1 / 8) <= 65535) {
        T1CONbits.TCKPS = 0b01;         // Prescaler 1:8
        PR1 = (uint16_t)(temp_pr1 / 8);
    } 
    else if ((temp_pr1 / 64) <= 65535) {
        T1CONbits.TCKPS = 0b10;         // Prescaler 1:64
        PR1 = (uint16_t)(temp_pr1 / 64);
    } 
    else {
        T1CONbits.TCKPS = 0b11;         // Prescaler 1:256 (División máxima)
        PR1 = (uint16_t)(temp_pr1 / 256);
    }

    T1CONbits.TON = 1;                         // Reactivar el Timer 1 con los nuevos parámetros físicos
    estado_actual.sistema_configurado = 1;     // Activar bandera para transicionar el estado del sistema
}

// Configuración a bajo nivel de los registros del Timer 1
void config_timer1(void){
    T1CON                 = 0;     // Reiniciar el registro de control
    T1CONbits.TON         = 0;     // Asegurar estado de apagado inicial
    IFS0bits.T1IF = 0;             // Limpiar banderas de interrupción pendientes
    IEC0bits.T1IE = 1;             // Habilitar las interrupciones por hardware para el Timer 1
    IPC0bits.T1IP = 5;             // Asignar nivel de prioridad 5 (crítico para la captura de datos)
    TMR1          = 0;             // Poner a cero el contador
    PR1           = 14745;         // Carga de valor por defecto inicial
}

// Configuración a bajo nivel del módulo ADC (Convertidor Analógico a Digital de 10 bits)
void config_adc(){
    ADCON1bits.ADON = 0;    // Apagar el módulo para escritura segura de registros
    ADPCFG = 0xFFCC;        // Configurar los pines AN0, AN1, AN4, y AN5 como entradas analógicas
    ADCON1bits.FORM = 0;    // Formato de salida configurado como enteros sin signo (0 a 1023)
    ADCON1bits.SSRC = 0;    // Disparo de conversión configurado por software (SAMP = 0)
    ADCON1bits.ASAM = 0;    // Inicio de muestreo manual
    ADCON2bits.VCFG = 0;    // Referencias de voltaje configuradas a rieles estándar (AVDD 5V y AVSS GND)
    ADCON2bits.CSCNA = 0;   // Deshabilitar escaneo automático de canales
    ADCON2bits.SMPI = 0;    // Interrupción después de cada conversión individual (no utilizada activamente)
    ADCON2bits.ALTS = 0;    // Utilizar únicamente el Mux A
    ADCON3bits.ADRC = 0;    // Utilizar el reloj derivado del sistema para las conversiones
    ADCON3bits.SAMC = 15;   // Asignar 15 ciclos TAD para la carga del capacitor Hold
    ADCON3bits.ADCS = 20;   // Aplicar divisor al reloj principal para estabilizar la etapa analógica
    ADCON1bits.ADON = 1;    // Habilitar oficialmente el módulo ADC
}

// Función de lectura manual (polling) para adquirir el dato de un canal específico
uint16_t leer_ADC(uint8_t canal){
    ADCHSbits.CH0SA = canal;  // Enrutar el multiplexor analógico al canal seleccionado
    ADCON1bits.SAMP = 1;      // Cerrar el switch interno para cargar el capacitor de retención
    __delay_us(10);           // Tiempo de estabilización física del voltaje
    ADCON1bits.SAMP = 0;      // Abrir el switch e iniciar la conversión digital
    
    while(!ADCON1bits.DONE);  // Bucle de espera hasta que el hardware complete el cálculo
    return ADCBUF0;           // Extraer y devolver el valor del buffer de lectura
}