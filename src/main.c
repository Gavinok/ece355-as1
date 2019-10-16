//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------
#define STM32F051
#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"


/* these are used to make lsp work better */
/* #include "../system/include/cmsis/stm32f0xx.h" */
/* #include "../include/diag/Trace.h" */

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);

// Your global variables...

uint16_t edgeCount = 0;

float signalFreq = 0.0;
float signalPer = 0.0;

/*
volatile uint32_t AHBENR;

#define RCC_BASE              (AHBPERIPH_BASE + 0x00001000)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)

#define PERIPH_BASE         ((uint32_t)0x40000000)
#define RCC_AHBENR_GPIOA	((uint32_t))0x00000000)
#define GPIOA_BASE          (AHB2PERIPH_BASE + 0x00000000)
#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE)
*/


int
main(int argc, char* argv[])
{

	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */

	while (1)
	{
		// Nothing is going on here...
	}

	return 0;

}


void myGPIOA_Init()
{
	trace_printf("myGPIOA_Init\n");

	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
	trace_printf("myGPIOA_Init end\n");

}


void myTIM2_Init()
{
	trace_printf("myTIM2_Init\n");

	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER; /* given */
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD; /* given */

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

	/* not sure if we should do it this way but we could */
	/* TIM2->CR1 |= TIM_CR1_CEN; */

	trace_printf("myTIM2_Init end\n");

}


void myEXTI_Init()
{
	trace_printf("myEXTI_Init\n");
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	  EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	  NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	  NVIC_EnableIRQ(EXTI0_1_IRQn);
		trace_printf("myGPIOA_Init end\n");

}

/* --------------Do Not Mess With This-----------*/
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;


	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	trace_printf("EXTI0_1_IRQHandler\n");

	// Your local variables...
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		// 1. If this is the first edge:

		/* uint16_t timerEnable = (TIM2->CR1 & TIM_CR1_CEN); */
		//if( first edge )
		//{
		if(edgeCount == 1){

			TIM2->CNT = 0x00000000;//	- Clear count register (TIM2->CNT).
			TIM2->CR1 |= TIM_CR1_CEN;//	- Start timer (TIM2->CR1).
		}
		else{
		//    Else (this is the second edge):
		//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);
		//	- Read out count register (TIM2->CNT).
			uint32_t counter = TIM2->CNT;
		//	- Calculate signal period and frequency.
			signalFreq = ((float)SystemCoreClock)/counter;
			signalPer = 1/signalFreq;
		//	- Print calculated values to the console.
			trace_printf("Signal Freq:   %f Hz\n", signalFreq);
			trace_printf("Signal Period: %f s\n", signalPer);
		//	  NOTE: Function trace_printf does not work
		//	  with floating-point numbers: you must use
		//	  "unsigned int" type to print your signal
		//	  period and frequency.
		//
		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		//
		EXTI->PR |= EXTI_PR_PR1;
	}
	trace_printf("EXTI0_1_IRQHandler end\n");

}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
