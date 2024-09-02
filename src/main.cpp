#include <Arduino.h>
#include "avr8-stub.h"
#include "app_api.h" // only needed with flash breakpoints

using deg_t = int;
static constexpr auto TIMER1_PRESCALE = 8;

namespace util {
    template <typename timer_t, typename calc_t = uint32_t>
    constexpr timer_t time_to_ticks(calc_t const time_scaled, calc_t const prescale = 1, calc_t const cpu_frequency = F_CPU, calc_t const time_factor = 1) {
        return static_cast<timer_t>(
            (time_scaled * (cpu_frequency / prescale)) / time_factor);
    }
}

void set_servo_position(deg_t const deg) {
    static constexpr uint16_t pulse_range[] = {400, 2500};
    static constexpr deg_t degree_range[] = {0, 203};
    volatile auto pulse_width_us = (
        static_cast<int32_t>(deg - degree_range[0]) *
        (pulse_range[1] - pulse_range[0])
        ) / (degree_range[1] - degree_range[0]) + pulse_range[0];
    OCR1A = util::time_to_ticks<uint16_t, uint32_t>(pulse_width_us, TIMER1_PRESCALE, F_CPU, 1000000);
}

void setup() {
    debug_init();

    // Set Timer 1 to waveform generation mode 14 (Fast PWM, 16-bit)
    TCCR1A = _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12);
    // Set input capture TOP to 20 ms
    static constexpr auto TMR1_TOP = util::time_to_ticks<uint16_t, uint32_t>(20, TIMER1_PRESCALE, F_CPU, 1000);
    ICR1 = TMR1_TOP;

    // Set OCR1A on compare match, Clear OC1A at BOTTOM.
    TCCR1A |= _BV(COM1A1);

    // Set PORTB1 to output (OC1A)
    DDRB |= _BV(PORTB1);
    
    // Set prescale to 8 (also starts the timer)
    TCCR1B |= _BV(CS11);
    static_assert(TIMER1_PRESCALE == 8, "Incorrect timer 1 prescale set in hardware");
}

void loop() {
    auto v = (static_cast<uint32_t>(analogRead(A0)) * 180) / 1024;
    set_servo_position(v);
}
