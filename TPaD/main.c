/* 
 * File:   main.c
 * Author: Motoki Mizoguchi based on code from David Meyer
 *
 * Created on August 22nd, 2012
 *
 * Editor: Craig Shultz
 *
 * Edited on November 21st, 2012
 * Added USB Host functionality and removed need for ADC sampling and converstion
 * TPad now interfaces with Kindle Fire via USB and the Android Debug Port Bridge (ADB)
 */

// DEVCFG3:
#pragma config IOL1WAY  = OFF       // Peripheral Pin Select Configuration, allow mult reconfig
#pragma config PMDL1WAY = OFF       // Peripheral Module Disable Config, allow mult reconfig
// DEVCFG2:
#pragma config UPLLEN   = ON        // USB PLL Enable
#pragma config UPLLIDIV = DIV_2     // USB PLL Divider
// 8MHz / 2 * 24 / 2 = 48MHz
#pragma config FPLLODIV = DIV_2     // PLL Output Divider
#pragma config FPLLMUL  = MUL_20    // PLL Multiplier
#pragma config FPLLIDIV = DIV_2     // PLL Input Divider
// 8MHz / 2 * 20 / 2 = 40MHz

// DEVCFG1:
#pragma config FWDTEN   = OFF       // Watchdog Timer
#pragma config WDTPS    = PS1       // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD    // Clock Switching & Fail Safe Clock Monitor
#pragma config FPBDIV   = DIV_1     // Peripheral Clock divisor
#pragma config OSCIOFNC = OFF       // CLKO Enable
#pragma config POSCMOD  = HS        // Primary Oscillator
#pragma config IESO     = OFF       // Internal/External Switch-over
#pragma config FSOSCEN  = OFF       // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL    // Oscillator Selection
// DEVCFG0:
#pragma config CP       = OFF       // Code Protect
#pragma config BWP      = ON        // Boot Flash Write Protect
#pragma config PWP      = OFF       // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx1  // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF       // JTAG Enable
#pragma config DEBUG    = OFF       // Background Debugger Enable


/** Includes *******************************************************/
#include <plib.h>
#include "adb.h"
#include "connection.h"
#include "HardwareProfile.h"

/** PreCompiler Definitions ****************************************/
#define SAMPLE_TIME 10	// 10 core timer ticks = 250 ns
#define DEBUGLED LATBbits.LATB7
#define OUTPUT_FREQ 33530
#define COMMAND_MAG 0x0a
#define COMMAND_FREQ 0x0b

/** Global Variables ***********************************************/
int tpadMAG;
int tpadFREQ;
int count = 0;
int incoming = 0;

typedef enum {
    STATE_INIT,
    STATE_WAITING,
    STATE_CONNECTING,
    STATE_CONNECTED
} STATE;

static CHANNEL_HANDLE h;
static STATE state = STATE_INIT;

/** Function Headers ***********************************************/
void PWM_Set_DC(double dc);
void set_timer2(int freq);
void init_PWM(void);
void PIC32MX250_setup_pins(void);

// ADB communication channels
void ADBCallback(ADB_CHANNEL_HANDLE handle, const void* data, UINT32 data_len);
// Convert between bytes and integers
int intFromBytes(BYTE in[]);

union {
    BYTE array[4];
    int value;
} iConverter;

int main() {
    CFGCONbits.JTAGEN = 0; // turn off JTAG, get back those pins for IO use
    // set PIC32to max computing power
    DEBUGLED = 0;
    // enable multi-vector interrupts
    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();
    PIC32MX250_setup_pins();
    SYSTEMConfigPerformance(SYS_FREQ);

    set_timer2(OUTPUT_FREQ); //
    init_PWM(); //Initializing PWM on OC3.

    // Initialize the USB host
    ConnectionInit();
    DEBUGLED = 1;

    //Main USB State Machine
    while (1) {
        // Keep the USB connection running;
        // Handle incoming data and manage outgoing data.
        ConnectionTasks();
        // Main state machine
        switch (state) {
            case STATE_INIT:

                state = STATE_WAITING;
                h = INVALID_CHANNEL_HANDLE;
                break;

            case STATE_WAITING:
                DEBUGLED = 0;
                if (ADBAttached()) {
                    state = STATE_CONNECTING;
                }
                break;

            case STATE_CONNECTING:

                if (ADBConnected()) {
                    // Open a channel to the Android device
                    // See "adb.h" in libadb for more details
                    // (I don't think the name tcp:4545 matters)
                    h = ADBOpen("tcp:4545", &ADBCallback);

                    if (h != INVALID_CHANNEL_HANDLE) {
                        state = STATE_CONNECTED;
                        // Send plaintext and let the recipient do formatting
                        ADBWrite(h & 0xFF, "Hello from TPAD!", 17);
                    }
                }
                break;

            case STATE_CONNECTED:
                DEBUGLED = 1;
                if (!ADBAttached()) {
                    state = STATE_INIT;
                }
                if (ADBChannelReady(h)) {
                    // Execute tasks that rely on the Android-PIC32 connection
                    // Here we will just wait for messages to come in and be handled below
                }
                break;
        } // end state machine
    } // end while loop

    return 0;
}

// Timer2 Set up function.

void set_timer2(int freq) {
    T2CON = 0b000 << 4; // set prescaler to 1:1
    T2CONSET = 1 << 15; // turn Timer2 on
    T2CONCLR = 0x2; // set input to PBCLK (the default; all defaults fine)

    PR2 = (SYS_FREQ / freq) - 1; // set the period in period register 2
    TMR2 = 0; // reset the Timer2 count
}

// following is used for setting the duty cycle as a percent

void PWM_Set_DC(double dc) {
    if (dc > 50) //maximum duty cycle is 50%
        dc = 50;
    if (dc < 1) //minimum would be 1%
        dc = 1;
    // Convert the percent to a number of counts
    OC3RS = (int) ((dc * (PR2 + 1)) / 100) - 1;
}

//PWM Initialization function.

void init_PWM(void) {
    OC3CONbits.OCM = 6; //PWM mode with no fault protection
    OC3RS = 0; // set buffered PWM duty cycle in counts
    OC3R = 0; // set initial PWM duty cycle in counts
    OC3CONbits.ON = 1; //Turn OC3 on.
    PWM_Set_DC(0); //Set duty cycle to 0%
};


// ===========================
// Helper function definitions
// ===========================

// Print any data received.
// This function is automatically called when data is received over ADB.
// In this demo, we assume all messages are to be printed.
// In your applications, you may want to create a system to classify data
// and process it appropriately (e.g. using the first byte to classify data)

void ADBCallback(ADB_CHANNEL_HANDLE handle, const void* data, UINT32 data_len) {
    // Ignore empty messages
    if (data_len < 1) {
        return;
    }
    //DEBUGLED = 0;
  
   // Get data without the first byte
    BYTE raw_data[data_len-1];
    memcpy(raw_data, data+1, data_len-1);

    // First byte identifies the data
    BYTE* id = (BYTE*) data;


    switch (*id) {

        case COMMAND_FREQ:
            incoming = intFromBytes(raw_data);
            PR2 = (SYS_FREQ /incoming) - 1;
            TMR2 = 0; // reset the Timer2 count
            break;
        case COMMAND_MAG:
            tpadMAG = intFromBytes(raw_data);
            PWM_Set_DC(tpadMAG / 1024.0 * 47);
            break;

    }


}

// Convert an array of four bytes to an integer

int intFromBytes(BYTE in[]) {
    iConverter.array[0] = in[0];
    iConverter.array[1] = in[1];
    iConverter.array[2] = in[2];
    iConverter.array[3] = in[3];
    return iConverter.value;
}
