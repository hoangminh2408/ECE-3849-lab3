/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>
#include <math.h>
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c1294ncpdt.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "oscilloscope.h"
#include "comparator.h"
#include "PWM.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "sysctl_pll.h"
#include "driverlib/udma.h"
#include "PWM.h"

//......DEFINES......//
#define ADC_OFFSET 2048
#define ADC_BUFFER_SIZE 2048 // size must be a power of 2
#define ADC_BUFFER_WRAP(i) ((i) & (ADC_BUFFER_SIZE - 1)) // index wrapping macro
#define ADC_BITS 12
#define VIN_RANGE 3.3
#define PIXELS_PER_DIV 20
#define ADC_BITS 12
#define ADC_OFFSET 2048
#define SYSTEM_CLOCK_MHZ 120            // [MHz] system clock frequency
#define EVENT1_PERIOD 5
#define TIMER1_PERIOD (SYSTEM_CLOCK_MHZ * EVENT1_PERIOD)
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required

//......GLOBALS......//
//volatile uint32_t gADCBufferIndex = ADC_BUFFER_SIZE - 1;  // Latest sample index
volatile uint16_t gADCBuffer[ADC_BUFFER_SIZE];            // Circular buffer
volatile uint16_t gSpectrum[1024];
volatile uint16_t spectrumDraw[1024];
volatile uint32_t gADCErrors = 0;
volatile uint32_t ADCIndex;
volatile uint16_t triggerDirection = 0;
volatile uint32_t triggerVoltage = 2048;
volatile uint16_t samplingRateState = 11;
volatile uint16_t mode = 1; // Mode 1: Oscilloscope, Mode 0: Spectrum
volatile float VOLTS_PER_DIV = 0.5;
volatile uint16_t ADCDraw[128];
volatile uint16_t ADCDrawScaled[128];
uint32_t gSystemClock; // [Hz] system clock frequency
volatile uint32_t gTime = 8345; // time in hundredths of a second
volatile uint16_t voltageScaleState = 2;
volatile bool gDMAPrimary = true;
uint32_t buttonResponseTime = 0;
uint32_t buttonLatency = 0;
uint32_t buttonMissedDeadlines = 0;
uint32_t count_unloaded = 0;
uint32_t count_loaded = 0;
float cpu_load = 0.0;
char string[30];
char string2[30];
uint32_t gSystemClock = 120000000; // [Hz] system clock frequency
uint32_t gpressed = 0;
tDMAControlTable gDMAControlTable[64];     // uDMA control table (global)
volatile uint32_t lastTime = 0;
volatile uint32_t difference = 0;
volatile uint32_t periodInterval = 0;
volatile uint32_t periods = 0;
float cyclesPerPeriod;
float averagePeriod;

////......FUNCTION PROTOTYPES......//
void ADCInit(void);
void initMain();
int32_t getADCBufferIndex(void);
uint32_t cpu_load_count(void);

#define PI 3.14159265358979f
#define NFFT 1024 // FFT length
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state)+sizeof(kiss_fft_cpx)*(NFFT-1))

//......MAIN......//
int main(void)
{
    IntMasterDisable();
    initMain();
    initComparator();
    /* Start BIOS */
    BIOS_start();
    return (0);
}

void ClockTask(UArg arg1, UArg arg2)
{
    Semaphore_post(Button_Semaphore); // Post to Button Semaphore
}

void FrequencyClockTask(UArg arg1, UArg arg2)
{
    Semaphore_post(Frequency_Semaphore); // Post to Frequency Semaphore
}

void FrequencyTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();

    initPWM();

    while (true)
    {
        Semaphore_pend(Frequency_Semaphore, BIOS_WAIT_FOREVER); // Pends for frequency semaphore
        IArg key = GateHwi_enter(gateHwi1);
        cyclesPerPeriod = (float) periodInterval / (float) periods; // Find clock cycles per period
        averagePeriod = (cyclesPerPeriod / gSystemClock); // Find the period
        periodInterval = 0;
        periods = 0;
        GateHwi_leave(gateHwi1, key);
    }
}
void ButtonTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    uint32_t t;
    uint32_t timer0_period = TimerLoadGet(TIMER0_BASE, TIMER_A) + 1;
    while (true)
    {
        Semaphore_pend(Button_Semaphore, BIOS_WAIT_FOREVER);
        t = timer0_period - TimerValueGet(TIMER0_BASE, TIMER_A);
        if (t > buttonLatency)
            buttonLatency = t; // Measure latency
        gpressed = getButton();
        if (gpressed != 0)
        {
            Mailbox_post(Button_Mailbox, &gpressed, BIOS_WAIT_FOREVER);
        }
        if (Semaphore_getCount(Button_Semaphore))
        { // Next event occurred
            buttonMissedDeadlines++;
            t = 2 * timer0_period; // Timer overflowed since last event
        }
        else
        {
            t = timer0_period;
        }
        t -= TimerValueGet(TIMER0_BASE, TIMER_A);
        if (t > buttonResponseTime)
        {
            buttonResponseTime = t;
        }
    }
}
void UserInputTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    while (true)
    {
        uint32_t presses;
        Mailbox_pend(Button_Mailbox, &presses, BIOS_WAIT_FOREVER);
        ButtonPress(presses);
        Semaphore_post(Display_Semaphore);
    }
}
void DisplayTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    tContext sContext;
    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontCmss12);     // Select font
    tRectangle rectFullScreen = { 0, 0, GrContextDpyWidthGet(&sContext) - 1,
    GrContextDpyHeightGet(&sContext) - 1 };
    while (true)
    {
        Semaphore_pend(Display_Semaphore, BIOS_WAIT_FOREVER);
        // Local variables to prevent shared data issues
        uint16_t voltageScaleStateLocal = voltageScaleState;
        uint16_t samplingRateStateLocal = samplingRateState;
        uint16_t triggerDirectionLocal = triggerDirection;
        if (mode)
        {
            // Fill screen with black
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &rectFullScreen);
            // Compute CPU load
            count_loaded = cpu_load_count();
            cpu_load = 1.0f - (float) count_loaded / count_unloaded;
            // Convert CPU load to string for printing
            snprintf(string, sizeof(string), "CPU LOAD: %.02f %%",
                     cpu_load * 100);
            // Convert period to frequency and print
            snprintf(string2, sizeof(string2), "f = %.03f Hz",
                     1 / averagePeriod);
            // Draw all elements on the LCD screen
            drawWaveform(sContext, triggerDirectionLocal,
                         voltageScaleStateLocal, samplingRateStateLocal, string,
                         string2);
            // Draw waveform
            GrContextForegroundSet(&sContext, ClrYellow); // Yellow waveform
            int i;
            for (i = 0; i < 127; i++)
            {
                int currentY = ADCDrawScaled[i];
                int nextY = ADCDrawScaled[i + 1];
                GrLineDraw(&sContext, i, currentY, i + 1, nextY);
            }
            GrFlush(&sContext); // Update the LCD display
        }
        else
        {
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &rectFullScreen);
            drawSpectrum(sContext, samplingRateStateLocal);
            GrFlush(&sContext);
        }
    }
}
void WaveformTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    while (true)
    {
        Semaphore_pend(Waveform_Semaphore, BIOS_WAIT_FOREVER);
        if (mode)
        {
            uint16_t triggerDirectionLocal = triggerDirection;
            getWaveform(triggerDirectionLocal, triggerVoltage);
        }
        else
        {
            getSpectrum();
        }
        Semaphore_post(Processing_Semaphore); // Signal Processing task
    }
}
void ProcessingTask(UArg arg1, UArg arg2)
{
    IntMasterEnable();
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE]; // Kiss FFT config memory
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg; // Kiss FFT config
    static kiss_fft_cpx in[NFFT], out[NFFT]; // Complex waveform and spectrum buffers
    int i;
    static float w[NFFT]; // Window function

    for (i = 0; i < NFFT; i++)
    {
        // Blackman window
        w[i] = 0.42f - 0.5f * cosf(2 * PI * i / (NFFT - 1))
                + 0.08f * cosf(4 * PI * i / (NFFT - 1));
    }

    cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size); // init Kiss FFT

    while (true)
    {
        Semaphore_pend(Processing_Semaphore, BIOS_WAIT_FOREVER);
        if (mode)
        {
            int i;
            float VOLTS_PER_DIV_LOCAL = VOLTS_PER_DIV;
            float fScale = (VIN_RANGE * PIXELS_PER_DIV)
                    / ((1 << ADC_BITS) * VOLTS_PER_DIV_LOCAL);
            for (i = 0; i < 128; i++) // Calculate y coordinate of each point in the buffer
            {
                ADCDrawScaled[i] = LCD_VERTICAL_MAX / 2
                        - (int) roundf(fScale * (ADCDraw[i] - ADC_OFFSET));
            }
        }
        else
        {
            for (i = 0; i < NFFT; i++)
            { // generate an input waveform
                in[i].r = gSpectrum[i] * sinf(20 * PI * i / NFFT) * w[i]; // real part of waveform
                in[i].i = 0; // imaginary part of waveform
            }
            kiss_fft(cfg, in, out); // compute FFT
            for (i = 0; i < NFFT; i++)
            {
                // Calculate magnitude = sqrt(r^2+i^2), then convert to dB
                float dBMagnitude = log10f(
                sqrtf(out[i].r * out[i].r + out[i].i * out[i].i));
                spectrumDraw[i] = 120 - (int) roundf(17 * dBMagnitude); // Round values, and apply an offset so they will accurately display on screen. convert to decibel
            }
        }
        Semaphore_post(Display_Semaphore); // Post to Display_Semaphore
        Semaphore_post(Waveform_Semaphore); // Post to Waveform_Semaphore after DisplayTask is finished
    }
}

// Initialize hardware
void initMain()
{
    IntMasterDisable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_A_PERIODIC);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock / 100 - 1); // .01 sec interval
    count_unloaded = cpu_load_count();
    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();
    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                                      120000000);
    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation
    ButtonInit(); //Initialize buttons
    ADCInit(); //Initialize ADC
    IntMasterEnable();
}

uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}
