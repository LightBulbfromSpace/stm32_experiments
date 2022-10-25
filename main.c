#include <stdint.h>
#include <utils.h>
#include <stm32f10x.h>



int __attribute((noreturn)) main(void) {
	// Enable clock for AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	// Enable PC13 push-pull mode
	GPIOB->CRH &= ~GPIO_CRH_CNF12; //clear cnf bits
	GPIOB->CRH |= GPIO_CRH_MODE12_0; //Max speed = 10Mhz

    while (1) {
	    GPIOB->ODR |= (1U<<12U); //U -- unsigned suffix (to avoid syntax warnings in IDE)
		delay(1000000);
	    GPIOB->ODR &= ~(1U<<12U);
	    delay(1000000);
    }
}
