#include "project.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "qcw.h"
#include "DMA.h"
#include "ZCDtoPWM.h"
#include "interrupter.h"
#include "telemetry.h"
#include "helper/printf.h"
#include <device.h>
#include "qcw.h"
#include "cli_common.h"
#include "alarmevent.h"
#include "tasks/tsk_analog.h"
#include "tasks/tsk_cli.h"
#include "tasks/tsk_fault.h"
#include "tasks/tsk_midi.h"
#include "tasks/tsk_thermistor.h"
#include "tasks/tsk_uart.h"
#include "tasks/tsk_usb.h"
#include "tasks/tsk_min.h"
#include "tasks/tsk_display.h"
#include "tasks/tsk_hwGauge.h"
#include "tasks/tsk_duty.h"
#include "tasks/tsk_hypervisor.h"
#include <stdio.h>

/*
 * Installs the RTOS interrupt handlers and starts the peripherals.
 */
#include "ADC.h" // Include ADC header (assuming this is the ADC used)

// Constants
#define POT_THRESHOLD 15    // ADC value corresponding to 2k ohms
#define POT_MAX 254         // Maximum ADC value for 10k ohms potentiometer

#define STEP_SIZE 1

static void prvFreeRTOSSetup(void);


volatile uint8_t qcwState = 0;






void PotentiometerTask(void *pvParameters) {
    uint16_t ema = 0;                 // simple low-pass (8-bit ADC in 0..254)
    const uint8_t ON_TH  = 15;        // ~2kΩ (your mapping)
    const uint8_t OFF_TH = 10;         // hysteresis: turn off below this

    for (;;) {
        uint8_t raw = ADC_Pot_GetResult8();     // or ADC_Pot_Read() on your part
        ema = ema + ((raw - ema) >> 2);         // ~1/4 smoothing

        // Hysteretic state machine
        if (!qcwState && ema >= ON_TH) {
            qcw_start();
            qcwState = 1;
        } else if (qcwState && ema < OFF_TH) {
            qcw_stop();
            qcwState = 0;
        }

        // Visualize the pedal/OCD level (8-bit)
        OCD_LEVEL_LED_WriteCompare((uint8_t)ema);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void Initialize() {
    CyGlobalIntEnable; // Enable global interrupts.
    OCD_LEVEL_LED_Start();
   

    // ADC Initialization
    ADC_Pot_Start();
    ADC_Pot_StartConvert();
}

int main() {
    Initialize();
    
    relay_write_bus(0);
    relay_write_charge_end(0);
    
    prvFreeRTOSSetup();
    alarm_init();
    sysflt_set(pdFALSE); //this should suppress any start-up sparking until the system is ready
    init_config();
    EEPROM_1_Start();
    SG_Timer_Start();
    
    null_port.type = PORT_TYPE_NULL;
    null_port.tx = NULL;
    null_port.rx = NULL;
    
    null_handle->port = &null_port;
    null_handle->print = stream_printf;
    
    eeprom_load(null_handle);
    
    initialize_DMA();          //sets up all DMA channels
    initialize_interrupter();  //initializes hardware related to the interrupter
    initialize_ZCD_to_PWM();   //initializes hardware related to ZCD to PWM
    initialize_charging();
    
    //calls that must always happen after updating the configuration/settings
    configure_ZCD_to_PWM();
    
    LED_com_Write(LED_ON);
    
    // Starting Tasks
    if (configuration.minprot) {
        tsk_min_Start();        //Handles UART-Hardware and queues with MIN-Protocol
    } else {
        tsk_uart_Start();       //Handles UART-Hardware and queues
    }
    
    tsk_usb_Start();        //Handles USB-Hardware and queues
    tsk_cli_Start();        //Commandline interface
    tsk_midi_Start();       //MIDI synth
    tsk_analog_Start();     //Reads bus voltage and currents
    tsk_fault_Start();      //Handles fault conditions
    tsk_duty_Start();
    
    if (configuration.enable_display) {
        tsk_display_Start();
    }
    if (configuration.pca9685) {
        tsk_hwGauge_init();
    }
    
    tsk_hypervisor_Start();
    alarm_push(ALM_PRIO_INFO, "INFO: UD3 startup", ALM_NO_VALUE);
    
    // Create the task for handling Quad Decoder and DAC update
    
    xTaskCreate(PotentiometerTask, "PotentiometerTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    // If all is well, this will never be reached
    for (;;) {
        // Infinite loop
    }

    
}

void prvFreeRTOSSetup(void) {
    /* Port layer functions that need to be copied into the vector table. */
    extern void xPortPendSVHandler(void);
    extern void xPortSysTickHandler(void);
    extern void vPortSVCHandler(void);
    extern cyisraddress CyRamVectors[];

    /* Install the OS Interrupt Handlers. */
    CyRamVectors[11] = (cyisraddress)vPortSVCHandler;
    CyRamVectors[14] = (cyisraddress)xPortPendSVHandler;
    CyRamVectors[15] = (cyisraddress)xPortSysTickHandler;

    /* Start-up the peripherals. */
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    /* The stack space has been exceeded for a task, consider allocating more. */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vApplicationMallocFailedHook(void) {
    /* The heap space has been exceeded. */
    taskDISABLE_INTERRUPTS();
    for (;;);
}

void vConfigureTimerForRunTimeStats(void) {
    // Configure timer for runtime stats
}

/* [] END OF FILE */
