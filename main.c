#include <stdint.h>
#include <utils.h>
#include <stm32f10x.h>

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
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PC14 Pull-up (for DOWN)

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
                ledPeriod += 1000;
            }
            if (GPIOA->IDR & 1) {
                ledPeriod -= 1000;
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

	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0;

	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14) | GPIO_CRH_CNF14_1;
    GPIOC->ODR |= GPIO_ODR_ODR14; //enable PC14 Pull-up (for MID)

	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15) | GPIO_CRH_CNF15_1;
    GPIOC->ODR |= GPIO_ODR_ODR15; //enable PC15 Pull-up (for UP)

	GPIOA->CRL = GPIOA->CRL & ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0) | GPIO_CRL_MODE0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0; //enable PC14 Pull-up (for DOWN)

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 35999;						// tick of timer is one millisecond
	TIM2->ARR = 1000;						// 
	TIM2->DIER = TIM_DIER_UIE;				// update interrupt enable 
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;

	_Bool btn_prev_state = 0;
	while(1) {
		// if (GPIOC->IDR & (1 << 15U)) {
        //         TIM2->ARR += 100;
        // }
		//  if (GPIOA->IDR & 1) {
        //         TIM2->ARR -= 100;
		// }
		_Bool btn_state = !(GPIOC->IDR & (1 << 14U));
		if (btn_state && !btn_prev_state) {
			TIM2->CR1 &= ~TIM_CR1_CEN;	//stop
		} else {
			TIM2->CR1 |= TIM_CR1_CEN;	//start
		}
		btn_prev_state = btn_state;
		delay_us(10000); // 10ms
	}
	#endif
	#if 1
	    // Enable Port B, C
	    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
	    // Enable clock for USART3
	    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	    // RB11 = RX->Input pull-up
	    // RB10 = TX->ALternate func. push-pull
	    GPIOB->CRH = GPIOB->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10) | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_0;
	    GPIOB->CRH = GPIOB->CRH & ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11) | GPIO_CRH_CNF11_1;
	    GPIOB->ODR |= GPIO_ODR_ODR11;

	    // Baud rate = 9600

	    USART3->BRR = (1875 << 4);
	    // Enable USART, USART receiver, USART transmitter
	    USART3->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	    char* str[] = {'D', 'I', 'E'};
	    while (1)
	    {
		      GPIOC->ODR |= (1U<<13U);
		      delay(1000000);
		      GPIOC->ODR &= ~(1U<<13U);
		      for (uint8_t i = 0; i < sizeof(str); i++)
		      {
			USART3->DR = str[i];
			while (!(USART3->SR & USART_SR_TXE));
		      }
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
