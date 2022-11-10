#include <utils.h>
#include <stm32f10x.h>

void delay(uint32_t ticks) {
    //for (int i=0; i<ticks; i++) {
    //    __NOP();
    //}
    __asm__ volatile(
        "mov r0, #0x0\n\t"
        "mov r1, %0\n\t"
        "loop:\n\t"
            "add r0, r0, #1\n\t"
            "cmp r0, r1\n\t"
        "bne loop\n\t"
        :: "r"(ticks)
    );
}


void delay_us(uint32_t us) {
    __asm volatile (
        "push {r0}\r\n"
        "mov R0, %0\r\n"
        "_loop:\r\n"                //approx. 8 ticks/iteration
            "cmp R0, #0\r\n"        //
            "beq _exit\r\n"         //1 or 1+P (if true: time for clear pipeline)
            "sub R0, R0, #1\r\n"    //1
            "nop\r\n"               //1 alignment
            "b _loop\r\n"           //1+P (pipleline)
        "_exit:\r\n"
        "pop {r0}\r\n"
        :: "r" (9 * us)             //for 72 Mhz
    );
}

void SPI1_Init(void)
{
  //Включаем тактирование SPI1 и GPIOA
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_IOPAEN;
  
  /**********************************************************/
  /*** Настройка выводов GPIOA на работу совместно с SPI1 ***/
  /**********************************************************/
  //PA7 - MOSI (=SI Slave In)
  //PA6 - MISO (Free)
  //PA5 - SCK (=SCL Clock)

  // Also used with the display
  //PA4 - CS (=CS Chip Select)
  //PA3 - RS (also called A0, data=1 or command=0)
  //PA2 - RSE (Reset=0, Standby=1)
  
  //Для начала сбрасываем все конфигурационные биты в нули
  GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2 
                | GPIO_CRL_CNF3 | GPIO_CRL_MODE3
                | GPIO_CRL_CNF4 | GPIO_CRL_MODE4
				        | GPIO_CRL_CNF5 | GPIO_CRL_MODE5 
                | GPIO_CRL_CNF6 | GPIO_CRL_MODE6
                | GPIO_CRL_CNF7 | GPIO_CRL_MODE7);

  //Настраиваем
  //SCK: MODE5 = 0x03 (11b); CNF5 = 0x02 (10b)
  GPIOA->CRL |= GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1;
  
  //MISO: MODE6 = 0x00 (00b); CNF6 = 0x01 (01b)
  GPIOA->CRL |= GPIO_CRL_CNF6_0;
  
  //MOSI: MODE7 = 0x03 (11b); CNF7 = 0x02 (10b)
  GPIOA->CRL |= GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1;

  //CS: MODE4 = 0x03 (11b); CNF4 = 0x00 (00b)
  GPIOA->CRL |= GPIO_CRL_MODE4;

  //RS: MODE3 = 0x03 (11b); CNF3 = 0x00 (00b)
  GPIOA->CRL |= GPIO_CRL_MODE3;

  //RESET: MODE2 = 0x03 (11b); CNF2 = 0x00 (00b)
  GPIOA->CRL |= GPIO_CRL_MODE2;
  GPIOA->ODR |= GPIO_ODR_ODR2; // Set 'RESET' high (Standby mode)

  /**********************/
  /*** Настройка SPI1 ***/
  /**********************/
  
  SPI1->CR1 &= ~SPI_CR1_DFF; // DFF=0 Размер кадра 8 бит
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // LSBFIRST=0 MSB First
  SPI1->CR1 &= ~SPI_CR1_CRCEN; // Disable CRC
  SPI1->CR1 |= SPI_CR1_SSM; // Программное управление SS
  SPI1->CR1 |= SPI_CR1_SSI; // SS в высоком состоянии
  SPI1->CR1 |= SPI_CR1_BR;
  SPI1->CR1 |= SPI_CR1_MSTR; // Режим Master (ведущий)
  SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA); //Режим работы SPI: CPOL=0 CPHA=0
  
  SPI1->CR1 |= SPI_CR1_SPE; //Включаем SPI
}

void SPI1_Write(uint8_t data)
{
    //Ждем, пока не освободится буфер передатчика
    while(!(SPI1->SR & SPI_SR_TXE));
    //заполняем буфер передатчика
    SPI1->DR = data;
    }

    uint8_t SPI1_Read(void)
    {
    SPI1->DR = 0; //запускаем обмен
    //Ждем, пока не появится новое значение
    //в буфере приемника
    while(!(SPI1->SR & SPI_SR_RXNE));
    //возвращаем значение буфера приемника
    return SPI1->DR;
}

void cmd(uint8_t data){ // Отправка команды
	GPIOA->ODR &= ~GPIO_ODR_ODR3; // A0=0 --указание на то, что отправляем команду
	GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0 – указание дисплею, что данные адресованы ему
	delay(1000);
	SPI1_Write(data);
	GPIOA->ODR |= GPIO_ODR_ODR4; // CS=1 – окончание передачи данных
}

void dat(uint8_t data){ // Отправка данных
	GPIOA->ODR |= GPIO_ODR_ODR3; // A0=1 --указание на то, что отправляем данные
	GPIOA->ODR &= ~GPIO_ODR_ODR4; // CS=0 – указание дисплею, что данные адресованы ему
	delay(1000);
	SPI1_Write(data);
	GPIOA->ODR |= GPIO_ODR_ODR4; // CS=1 – окончание передачи данных
}