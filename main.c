#include <stdint.h>
#include <utils.h>
#include <stm32f10x.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
//#include <stdlib.h>

#define min(a,b) ((a) < (b) ? (a) : (b))



int __attribute((noreturn)) main(void) {
	#if 0
	// Enable clock for AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    // Enable clock for GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz
									 //General purpose Push-pull [00]
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14) | GPIO_CRH_MODE14_1;
    GPIOC->ODR |= GPIO_ODR_ODR14; //enable PC14 Pull-up (for MID)

    GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15) | GPIO_CRH_MODE15_1;
    GPIOC->ODR |= GPIO_ODR_ODR15; //enable PC15 Pull-up (for UP)

    GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PA0 Pull-up (for DOWN)

	uint32_t ledPeriod = 1000000;
	uint32_t btnPeriod = 10000;
	uint32_t ledPhase = ledPeriod;
	uint32_t btnPhase = btnPeriod;
	_Bool ledEnabled = 1;
	_Bool buttonPrevState = GPIOC->IDR & (1 << 13U);
    while (1) {
		uint32_t tau = min(btnPhase, ledPhase);
		delay_us(tau);
		ledPhase -= tau;
		btnPhase -= tau;
        if (btnPhase == 0) {
            if (GPIOC->IDR & (1 << 15U)) {
                ledPeriod += 2000;
            }
            if (GPIOA->IDR & 1) {
                ledPeriod -= 2000;
            }
			btnPhase = btnPeriod;
			_Bool btnNewState = !(GPIOC->IDR & GPIO_IDR_IDR14);
			if (!btnNewState && buttonPrevState) {
				ledEnabled = !ledEnabled;
			}
			buttonPrevState = btnNewState;
		}
		if (ledPhase == 0) {
			ledPhase = ledPeriod	// for (int i = 0; i < 64; i++)
	// 	for (int j = 0; j < 128; j++)
	// 		draw_point(i, j);;
			
			if (ledEnabled) {
				GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_ODR13) << 16) | ( ~GPIOC->ODR & GPIO_ODR_ODR13);
			}
			
		}
    }
	#endif
	#if 0
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	
	RCC->APB2RSTR |= RCC_APB1RSTR_TIM2RST;  // set periphery to default
	RCC->APB2RSTR &= ~RCC_APB1RSTR_TIM2RST; // turn off reset register

	GPIOA->CRH = GPIOA->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13) | GPIO_CRH_MODE13_0; //LED pin

	GPIOC->CRH = GPIOC->CRH & ~(	// for (int i = 0; i < 64; i++)
	// 	for (int j = 0; j < 128; j++)
	// 		draw_point(i, j);GPIO_CRH_CNF14 | GPIO_CRH_MODE14) | GPIO_CRH_MODE14_1;
    GPIOC->ODR |= GPIO_ODR_ODR14; //enable PC14 Pull-up (for MID)

	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15) | GPIO_CRH_MODE15_1;
    GPIOC->ODR |= GPIO_ODR_ODR15; //enable PC15 Pull-up (for UP)

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PC14 Pull-up (for DOWN)

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 35999;						// tick of timer is one millisecond
	TIM2->ARR = 100;						// 
	TIM2->DIER = TIM_DIER_UIE;				// update interrupt enable 
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	//TIM2->CR1 &= ~TIM_CR1_ARPE;

	TIM2->CR1 |= TIM_CR1_CEN;
	
	bool btn_prev_state = false;
	while(1) {
		// if (!(GPIOC->IDR & (1 << 15U))) {
		// 		//TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
        //         TIM2->PSC += 500;
		// 		//TIM2->CR1 |= TIM_CR1_CEN;	//start
        // }
		//  if (!(GPIOA->IDR & 1)) {
		// 		//TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
        //         TIM2->PSC -= 500;
		// 		//TIM2->CR1 |= TIM_CR1_CEN;	//start
		// }
		bool btn_state = !(GPIOC->IDR & (1 << 15U));
		if (btn_state && !btn_prev_state) {
			if (TIM2->CR1 & TIM_CR1_CEN) {
				TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
			} else {
				TIM2->CR1 |= TIM_CR1_CEN;	//start
			}
		}
		btn_prev_state = btn_state;
		delay_us(10000); // 10ms
	}
	#endif
	#if 0

	typedef struct 
	{
		GPIO_TypeDef* gpio_x;
		uint32_t pin_num
	} LED_pin;
	
	LED_pin LED_pins[] = {{GPIOA, GPIO_ODR_ODR5}, {GPIOB, GPIO_ODR_ODR0}, {GPIOB, GPIO_ODR_ODR10}}; 

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
35999	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5) | GPIO_CRL_MODE5_0;
	//LED_pins[0].gpio_x->ODR |= LED_pins[0].pin_num;
	GPIOB->CRL = GPIOB->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_0;
	GPIOB->CRH = GPIOB->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10) | GPIO_CRH_MODE10_0;

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PA0 Pull-up (for UP)

	uint8_t active_led = 0;
	uint8_t i = 0;
	bool LED_active = false;
    while (1) {
	    if (!(GPIOA->IDR & 1)) {
			LED_pins[i].gpio_x->ODR &= ~LED_pins[i].pin_num;
			i = (i++) % 3;
			LED_pins[i].gpio_x->ODR |= LED_pins[i].pin_num;
			delay_us(100000);
        }
		if (!(GPIOA->IDR & GPIO_ODR_ODR15)) {
			if (LED_active) {
				LED_pins[i].gpio_x->ODR &= ~LED_pins[i].pin_num;
			} else {
				LED_pins[i].gpio_x->ODR |= LED_pins[i].pin_num;
			}
			LED_active =! LED_active;GPIO_ODR_ODR13
			delay_us(1000);
        }
		//  if (!(GPIOA->IDR & (1<<14U))) {
		// 	GPIOB->ODR &= ~active_led;
		// 	active_led -= 5;
		// 	GPIOB->ODR |= active_led % 10;
		// }
    }
	#endif
	#if 1
	uint8_t LCD_Buff[8][128];
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~GPIO_CRH_CNF13; //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz
	GPIOB->CRH &= ~GPIO_CRH_CNF11; //clear cnf bits
	GPIOB->CRH |= GPIO_CRH_MODE11_0; //Max speed = 10Mhz

	SPI1_Init();
	GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0
	GPIOA->ODR &= ~GPIO_ODR_ODR2; // RESET=0
	delay(10000); // Wait for the power stabilized
	GPIOA->ODR |= GPIO_ODR_ODR2; // RESET=1
	delay(1000);

	cmd(0xA2); //LCD Drive set 1/9 bias
	cmd(0xA0); // RAM Address SEG Output normal
	cmd(0xC8); // Common outout mode selection
	cmd(0x28 | 0x07); // Power control mode
	cmd(0x20 | 0x05); // Voltage regulator
	cmd(0xA6); // Normal color, A7 = inverse color
	cmd(0xAF); // Display onlcd
	cmd(0x40 | 0x00); // Set start line address (Lines 0x00...0x3F)
	//memset(LCD_Buff, 0, sizeof(LCD_Buff));
	for(int k=0; k<=7; k++){ // Clear DRAM
		cmd(0xB0 | k); // Set Page 0 (Pages 0x00...0x0F)
		for(int i=0; i<=127; i++)
		dat(0x00);
		cmd(0xEE); // End writing to the page, return the page address back
	}

	// Setting the page address:
	//cmd(0xB0 | 0x00); // Set Page 0 (Pages 0x00...0x0F)
	//
	// Setting the line address:
	//cmd(0x10 | 0x00); // Set column address MSB (0x00...0x0F)
	//cmd(0x00); // Set column address LSB (0x00...0x0F)

	// for (int i = 0; i < 64; i++)
	// 	for (int j = 0; j < 128; j++)
	// 		draw_point(i, j);

	DrawChess();
  
    while (1) { // LED blinking
	    GPIOC->ODR |= (1U<<13U); //U -- unsigned suffix (to avoid syntax warnings in IDE)
		delay(1000000);
	    GPIOC->ODR &= ~(1U<<13U);
	    delay(1000000);
    }
	#endif
}

// void TIM2_IRQHandler(void)
// {
// 	if (TIM2->SR & TIM_SR_UIF) { // SR - status register
// 		uint16_t gpio = GPIOC->ODR;
// 		GPIOC->ODR = ~(gpio & GPIO_ODR_ODR13) & GPIO_ODR_ODR13;
// 		TIM2->SR &= ~TIM_SR_UIF;
// 	}
// }