#include <stdint.h>
#include <utils.h>
#include <stm32f10x.h>
#include <stdbool.h>

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
			ledPhase = ledPeriod;
			
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

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7) | GPIO_CRL_MODE7_0; //LED pin

	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14) | GPIO_CRH_MODE14_1;
    GPIOC->ODR |= GPIO_ODR_ODR14; //enable PC14 Pull-up (for MID)

	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15) | GPIO_CRH_MODE15_1;
    GPIOC->ODR |= GPIO_ODR_ODR15; //enable PC15 Pull-up (for UP)

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PC14 Pull-up (for DOWN)

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 35999;						// tick of timer is one millisecond
	TIM2->ARR = 1000;						// 
	TIM2->DIER = TIM_DIER_UIE;				// update interrupt enable 
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 &= ~TIM_CR1_ARPE;

	TIM2->CR1 |= TIM_CR1_CEN;
	

	bool tim2_turned_on = true;
	bool btn_prev_state = false;
	while(1) {
		if (!(GPIOC->IDR & (1 << 15U))) {
				//TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
                TIM2->PSC += 500;
				//TIM2->CR1 |= TIM_CR1_CEN;	//start
        }
		 if (!(GPIOA->IDR & 1)) {
				//TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
                TIM2->PSC -= 500;
				//TIM2->CR1 |= TIM_CR1_CEN;	//start
		}
		bool btn_state = !(GPIOC->IDR & (1 << 14U));
		if (btn_state && !btn_prev_state) {
			if (tim2_turned_on) {
				TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
			} else {
				TIM2->CR1 |= TIM_CR1_CEN;	//start
			}
			tim2_turned_on = !tim2_turned_on;
		}
		btn_prev_state = btn_state;
		delay_us(10000); // 10ms
	}
	#endif
	#if 1

	typedef struct 
	{
		uint32_t GPIOx_ODR;
		uint32_t pin_num
	} LED_pin;
	
	LED_pin LED_pins[] = {{GPIOA->ODR, GPIO_ODR_ODR5}, {GPIOB->ODR, GPIO_ODR_ODR0}, {GPIOB->ODR, GPIO_ODR_ODR10}}; 

	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5) | GPIO_CRL_MODE5_0;
	LED_pins[0].GPIOx_ODR &= ~LED_pins[0].pin_num;
	GPIOB->CRL = GPIOB->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_0;
	GPIOB->CRH = GPIOB->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10) | GPIO_CRH_MODE10_0;

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PA0 Pull-up (for UP)

	uint8_t active_led = 0;
	uint8_t i = 0;
    while (1) {
	    // if (!(GPIOA->IDR & 1)) {
		// 	LED_pins[i].GPIOx_ODR &= ~LED_pins[i].pin_num;
		// 	i = (i++) % 3;
		// 	LED_pins[i].GPIOx_ODR |= LED_pins[i].pin_num;
		// 	delay(1000);
        // }
		//  if (!(GPIOA->IDR & (1<<14U))) {
		// 	GPIOB->ODR &= ~active_led;
		// 	active_led -= 5;
		// 	GPIOB->ODR |= active_led % 10;
		// }
    }
	#endif
}

void TIM2_IRQHandler(void)
{
	if (TIM2->SR & TIM_SR_UIF) { // SR - status register
		GPIOC->BSRR = ((GPIOC->ODR & GPIO_ODR_ODR13) << 16) | (~GPIOC->ODR & GPIO_ODR_ODR13);
		TIM2->SR &= ~TIM_SR_UIF;
	}
}
