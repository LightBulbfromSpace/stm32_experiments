#include <stm32f10x.h>

void delay(uint32_t ticks);
void delay_us(uint32_t us);

void SPI1_Init(void);
void SPI1_Write(uint8_t data);
void cmd(uint8_t data); // Отправка команды
void dat(uint8_t data); // Отправка данных