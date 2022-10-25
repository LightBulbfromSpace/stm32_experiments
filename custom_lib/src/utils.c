#include <utils.h>
#include <stm32f10x.h>

void delay(uint32_t ticks) {
    /*for (int i=0; i<ticks; i++) {
        __NOP();
    }*/
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
