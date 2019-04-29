/*
 * comparator.c
 *
 *  Created on: Apr 25, 2019
 *      Author: mhle
 */

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
#include <comparator.h>
#include <math.h>
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
#include "driverlib/comp.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "oscilloscope.h"
#include "comparator.h"
#include "PWM.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "sysctl_pll.h"
#include "driverlib/udma.h"

extern volatile uint32_t lastTime;
extern volatile uint32_t difference;
extern volatile uint32_t periodInterval;
extern volatile uint32_t periods;

void initComparator()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
    ComparatorRefSet(COMP_BASE, COMP_REF_1_65V);
    ComparatorConfigure(
            COMP_BASE,
            1,
            COMP_OUTPUT_NORMAL | COMP_TRIG_NONE | COMP_INT_RISE | COMP_ASRCP_REF);
    // configure GPIO for comparator input C1- at BoosterPack Connector #1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_4);
    // configure GPIO for comparator output C1o at BoosterPack Connector #1 pin 15
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeComparatorOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_C1O);
    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff); // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff); // use maximum prescale value
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void ComparatorISR(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);
    uint32_t currentTime = TimerValueGet(TIMER0_BASE, TIMER_A); // Read captured timer count
    difference = (currentTime - lastTime) & 0xffffff; // Difference between current and previous timer count. AND with ffffff to take care of wraparound
    lastTime = currentTime;
    periodInterval += difference; //Add the difference into the total time elapsed after the last calculation of period
    periods++; //Increment the number of periods elapsed.
}
