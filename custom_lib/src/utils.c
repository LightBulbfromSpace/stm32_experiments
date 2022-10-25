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
        :: "r" (9 * us)             //for 72 Vhz
    );
}

