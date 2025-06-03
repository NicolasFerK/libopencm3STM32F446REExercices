#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>

#include "core/timer.h"
#include "core/gpio.h"
#include "core/gpio-pins.h"

#include "common-defines.h"

void tim2_isr(void)
{
    if (timer_get_flag(TIM2, TIM_SR_CC1IF))
    {
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        uint16_t compare_time = timer_get_counter(TIM2);
        timer_set_oc_value(TIM2, TIM_OC1, compare_time + 10000);

        gpio_set(LED_PORT, LED_PIN);
    }
}

void timer_setup(void)
{
    nvic_enable_irq(NVIC_TIM2_IRQ);
    rcc_periph_reset_pulse(RST_TIM2);

    // timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP); NÃO É OBRIGATORIO PQ QUANDO RESETA O CLOCK ELE JA É SETADO

    timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));

    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);

    timer_set_period(TIM2, 65535);
    timer_set_oc_value(TIM2, TIM_OC1, 2500);

    timer_enable_counter(TIM2);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}