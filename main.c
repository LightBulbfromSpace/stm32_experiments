#include <stdint.h>
#include <utils.h>
#include <stm32f10x.h>



int __attribute((noreturn)) main(void) {
	// Enable clock for AFIO
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	// Enable clock for GPIOC
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	// Enable PC13 push-pull mode
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13); //clear cnf bits
	GPIOC->CRH |= GPIO_CRH_MODE13_0; //Max speed = 10Mhz 
									 //General purpose Push-pull [00]
	GPIOC->CRH = GPIOC->CRH & ~(GPIO_CRH_CNF14 | GPIO_CRH_MODE14) | GPIO_CRH_MODE14_1;
	GPIOC->ODR |= GPIO_ODR_ODR14; //enable PC14 Pull-up
	uint8_t button_pushed = 0xFF;
    while (1) {
		if (!(GPIOC->IDR & GPIO_IDR_IDR14))
		{
			button_pushed = ~button_pushed;
			delay(300000);			  //Debouncing (for this code isn't necessary)
			while (!(GPIOC->IDR & GPIO_IDR_IDR14));
		}
		if (button_pushed) {
			GPIOC->ODR &= ~(1U<<13U); //GPIOC->ODR &= ~GPIO_ODR_ODR13;
									  //U -- unsigned suffix (to avoid syntax warnings in IDE)
			delay_us(1000000);
			GPIOC->ODR |= (1U<<13U);  //GPIOC->ODR |= GPIO_ODR_ODR13;
			delay_us(1000000);		 	  //for(uint32_t i = 0; i < 300000; i++) { __NOP(); }
		}
    }
}
