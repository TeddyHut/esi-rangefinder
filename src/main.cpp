#include <Arduino.h>
#include <avr/interrupt.h>
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

    // template <bool repeat = false, typename tick_t = uint32_t, typename... args>
    // class RunAtInterval {
    //     // using ticks_fn_t = tick_t (*)();
    //     using callback_fn_t = void (*)(args...);
    //     callback_fn_t callback_fn;
    //     args... args;

    //     tick_t last_update;
    // public:
    //     void update(tick_t const ticks) {
    //         callback_fn(args...);
    //     }
    //     RunAtInterval(ticks_fn_t const ticks_fn, callback_fn_t const callback_fn, args... args) : ticks_fn(ticks_fn), callback_fn(callback_fn), args(args...), last_update(ticks_fn()) {}
    // };
}

class HCSR04 {
    volatile static uint32_t cumulative_ticks;
    volatile static uint16_t rising_capture;
public:
    constexpr static auto TRIGGER_PIN = 11;
    constexpr static auto ECHO_PIN = 8;
    volatile static uint32_t pulse_length_ticks;
    static inline void interrupt_timer1_capt() {
        // If capture is rising edge
        if (TCCR1B & _BV(ICES1)) {
            cumulative_ticks = 0;
            rising_capture = ICR1;
            // Change to falling capture interrupt
            TCCR1B &= ~_BV(ICES1);
        }
        // If capture is falling edge
        else {
            pulse_length_ticks = cumulative_ticks + ICR1 - rising_capture;
            // Disable input capture and overflow interrupt
            TIMSK1 &= ~(_BV(ICIE1) | _BV(TOIE1));
        }
    }
    static inline void interrupt_timer1_ovf() {
        // When the timer reaches TOP add the time since the rising capture.
        cumulative_ticks += OCR1A - rising_capture;
        // Tf there is another overflow we just add TOP to cumulative_ticks.
        rising_capture = 0;
    }
    static void setup() {
        pinMode(ECHO_PIN, INPUT);
        pinMode(TRIGGER_PIN, OUTPUT);
        digitalWrite(TRIGGER_PIN, LOW);
    }
    static void send_pulse() {
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);
        // Capture on rising edge
        TCCR1B |= _BV(ICES1);
        // Enable input capture and overflow interrupt
        TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);
    }
};

volatile uint32_t HCSR04::cumulative_ticks = 0;
volatile uint16_t HCSR04::rising_capture = 0;
volatile uint32_t HCSR04::pulse_length_ticks = 0;

ISR(TIMER1_CAPT_vect) {
    HCSR04::interrupt_timer1_capt();
}

ISR(TIMER1_OVF_vect) {
    HCSR04::interrupt_timer1_ovf();
}

void set_servo_position(deg_t const deg) {
    static constexpr uint16_t pulse_range[] = {400, 2500};
    static constexpr deg_t degree_range[] = {0, 203};
    volatile auto pulse_width_us = (
        static_cast<int32_t>(deg - degree_range[0]) *
        (pulse_range[1] - pulse_range[0])
        ) / (degree_range[1] - degree_range[0]) + pulse_range[0];
    OCR1B = util::time_to_ticks<uint16_t, uint32_t>(pulse_width_us, TIMER1_PRESCALE, F_CPU, 1000000);
}

void setup() {
    // debug_init();

    // Set Timer 1 to waveform generation mode 15 (Fast PWM, 16-bit, OCR1A as top)
    TCCR1A = _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12);
    // Set input capture TOP to 20 ms
    static constexpr auto TMR1_TOP = util::time_to_ticks<uint16_t, uint32_t>(20, TIMER1_PRESCALE, F_CPU, 1000);
    OCR1A = TMR1_TOP;

    // Set OCR1B on compare match, Clear OC1B at BOTTOM.
    TCCR1A |= _BV(COM1B1);

    // Set PORTB2 to output (OC1B)
    DDRB |= _BV(PORTB2);
    
    // Set prescale to 8 (also starts the timer)
    TCCR1B |= _BV(CS11);
    static_assert(TIMER1_PRESCALE == 8, "Incorrect timer 1 prescale set in hardware");

    // Set up HCSR04
    HCSR04::setup();

    Serial.begin(9600);
}

void loop() {
    auto v = (static_cast<uint32_t>(analogRead(A0)) * 180) / 1024;
    set_servo_position(v);
    
    HCSR04::send_pulse();
    delay(500);
    Serial.println(HCSR04::pulse_length_ticks);
}
