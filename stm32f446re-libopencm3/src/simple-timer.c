#include "simple-timer.h"
#include "system.h"
void simple_timer_setup(simple_timer_t* timer, uint64_t wait_time, bool auto_reset)
{
    timer->wait_time = wait_time;
    timer->auto_reset = auto_reset;
    timer->target_time = system_get_ticks() + wait_time;
}

bool simple_timer_has_elapsed(simple_timer_t* timer)
{
    uint64_t now = system_get_ticks();
    bool has_elapsed = now >= timer->target_time;

    if(has_elapsed && timer->auto_reset)
    {
        uint64_t drift = now - timer->target_time;
        timer->target_time = (now + timer->wait_time) - drift; 
    }

    return has_elapsed;
}

void simple_timer_reset(simple_timer_t* timer)
{
    simple_timer_setup(timer, timer->wait_time, timer->auto_reset);
}