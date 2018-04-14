#include "delay.h"
#include "init.h"

static uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickCycleCounter = 0;
static volatile uint32_t usTicks = 0;
uint16_t frameCounter = 1;
time_flag TIME_FLAG;

void SysTick_Handler(void)
{
	sysTickCycleCounter = *DWT_CYCCNT;

	if ((frameCounter % COUNT_1000HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_1000hz=TRUE;
	}
	if ((frameCounter % COUNT_500HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_500hz=TRUE;
	}
	if ((frameCounter % COUNT_200HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_200hz=TRUE;
		TIME_FLAG.time_sub.flag_200_1hz=TRUE;
	}
	if ((frameCounter % COUNT_100HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_100hz=TRUE;
		TIME_FLAG.time_sub.flag_100_1hz=TRUE;
	}
	if ((frameCounter % COUNT_50HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_50hz=TRUE;
	}
	if ((frameCounter % COUNT_20HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_20hz=TRUE;
	}
	if ((frameCounter % COUNT_10HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_10hz=TRUE;
	}
	if ((frameCounter % COUNT_5HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_5hz=TRUE;
	}
	if ((frameCounter % COUNT_2HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_2hz=TRUE;
	}
	if ((frameCounter % COUNT_1HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_1hz=TRUE;
	}
	if ((frameCounter % COUNT_0_5HZ) == 0)
	{
		TIME_FLAG.time_sub.flag_0_5hz=TRUE;
	}

   	sysTickUptime++;
	frameCounter++;
    if (frameCounter > FRAME_COUNT)
    {
    	frameCounter = 1;
	}	
}
uint32_t micros(void)
{
    register uint32_t oldCycle, cycle, timeMs;

    do
    {
        timeMs = __LDREXW(&sysTickUptime);
        cycle = *DWT_CYCCNT;
        oldCycle = sysTickCycleCounter;
    }
    while (__STREXW(timeMs , &sysTickUptime));

    return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}

uint32_t millis(void)
{
    return sysTickUptime;
}
void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;

    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    // enable the CPU cycle counter
    DWT_CTRL |= CYCCNTENA;
}

void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = *DWT_CYCCNT;
    IWDG_Feed();
    for (;;)
    {
        register uint32_t current_count = *DWT_CYCCNT;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;

        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
int Get_Ms(unsigned long *count)
{
	count[0] = sysTickUptime;
	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Delay Milliseconds
///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}
void delay_nor(u16 time)
{
	u16 i=0;
	while(time--)
	{
		i=10;
		while(i--);
	}
}
