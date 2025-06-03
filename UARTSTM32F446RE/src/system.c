#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "core/system.h"

static volatile uint64_t ticks = 0;

void tim3_isr(void)
{
    if (TIM_SR(TIM3) & TIM_SR_UIF) {
        TIM_SR(TIM3) &= ~TIM_SR_UIF;
        ticks++;
    }
}   

// static void rcc_setup(void)
// {
//     //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
// }

void tim3_setup(void)
{
    rcc_periph_reset_pulse(RST_TIM3);

    timer_set_prescaler(TIM3, (CPU_FREQ / 1000) - 1);
    timer_set_period(TIM3, 1); 

    timer_enable_irq(TIM3, TIM_DIER_UIE); 
    nvic_enable_irq(NVIC_TIM3_IRQ); 

    timer_enable_counter(TIM3); 
}

uint64_t system_get_ticks(void)
{
    return ticks;
}

void system_setup(void)
{
    //rcc_setup();
    tim3_setup(); 
}

void system_teardown(void)
{
    timer_disable_counter(TIM3);
    timer_disable_irq(TIM3, TIM_DIER_UIE);
    nvic_disable_irq(NVIC_TIM3_IRQ);
}

void system_delay(uint64_t milliseconds)
{
    uint64_t end = system_get_ticks() + milliseconds;
    while (system_get_ticks() < end) {
       
    }
}
