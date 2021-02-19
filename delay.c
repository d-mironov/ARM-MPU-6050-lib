
#include "../mpu6050/delay.h"

// For store tick counts in us
static __IO uint32_t usTicks;

// SysTick_Handler function will be called every 1 us
void SysTick_Handler()
{
	if (usTicks != 0)
	{
		usTicks--;
	}
}

void DelayInit()
{
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Generate an interrupt every 1uS
	SysTick_Config(SystemCoreClock / 1000000);
}

void DelayUs(uint32_t us)
{
	// Reload us value
	usTicks = us;
	// Wait until usTick reach zero
	while (usTicks);
}

void DelayMs(uint32_t ms)
{
	// Wait until ms reach zero
	while (ms--)
	{
		// Delay 1ms
		DelayUs(1000);
	}
}
