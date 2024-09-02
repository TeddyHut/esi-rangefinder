#include <Arduino.h>
#include <avr/interrupt.h>
#include <arduino-timer.h>
#include "avr8-stub.h"
#include "app_api.h" // only needed with flash breakpoints

using deg_t = int;
using us_t = uint32_t;
using tick_t = uint32_t;
static constexpr auto TIMER1_PRESCALE = 8;
Timer<10, micros> timer;
using Timer_t = decltype(timer);

volatile bool reached_pos = false;

namespace util {
    template <typename timer_t, typename calc_t = uint32_t>
    constexpr timer_t time_to_ticks(calc_t const time_scaled, calc_t const prescale = 1, calc_t const cpu_frequency = F_CPU, calc_t const time_factor = 1) {
        return static_cast<timer_t>(
            (time_scaled * (cpu_frequency / prescale)) / time_factor);
    }
}

namespace servo {
    void setup() {
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
    }

    void set_position(deg_t const deg) {
        static constexpr uint16_t pulse_range[] = {400, 2500};
        static constexpr deg_t degree_range[] = {0, 203};
        volatile auto pulse_width_us = (
            static_cast<int32_t>(deg - degree_range[0]) *
            (pulse_range[1] - pulse_range[0])
            ) / (degree_range[1] - degree_range[0]) + pulse_range[0];
        OCR1B = util::time_to_ticks<uint16_t, uint32_t>(pulse_width_us, TIMER1_PRESCALE, F_CPU, 1000000);
    }
}

class HCSR04 {
    volatile static tick_t cumulative_ticks;
    volatile static uint16_t rising_capture;
public:
    constexpr static auto TRIGGER_PIN = 11;
    constexpr static auto ECHO_PIN = 8;
    static void (*falling_callback_fn)(tick_t);

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
            // Disable input capture and overflow interrupt
            TIMSK1 &= ~(_BV(ICIE1) | _BV(TOIE1));
            if (falling_callback_fn != nullptr)
                falling_callback_fn(cumulative_ticks + ICR1 - rising_capture);
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
        //delayMicroseconds(10);
        timer.in(10, [](auto) {
            digitalWrite(TRIGGER_PIN, LOW);
            // Capture on rising edge
            TCCR1B |= _BV(ICES1);
            // Enable input capture and overflow interrupt
            TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);
            return false;
        });
    }
};

volatile tick_t HCSR04::cumulative_ticks = 0;
volatile uint16_t HCSR04::rising_capture = 0;
void (*HCSR04::falling_callback_fn)(tick_t) = nullptr;

ISR(TIMER1_CAPT_vect) {
    HCSR04::interrupt_timer1_capt();
}

ISR(TIMER1_OVF_vect) {
    HCSR04::interrupt_timer1_ovf();
}

template <uint8_t avg_buf_len>
class RangeFinder {
private:
    using segment_t = unsigned int;
    static deg_t deg_min, deg_max;
    static deg_t inprogress_measurement_deg;
    static segment_t segment_count;
    static segment_t segment_idx;
    static int8_t direction;
    // Buffer probably not needed - should be able to create running average without storing previous values - TODO. Then could also be configurable at runtime.
    static tick_t movement_time_ms;
    volatile static tick_t pulse_length_buf[avg_buf_len];
    volatile static uint8_t buf_idx;
    struct ThreadFlags {
        bool schedule_next_position : 1;
        bool send_pulse : 1;
    } static volatile thread_flags;
public:
    struct Measurement {
        deg_t deg;
        tick_t pulse_length;
    } static volatile most_recent_measurement;

    static void set_degree_range(deg_t const min, deg_t const max) { deg_min = min; deg_max = max; }
    static void set_segment_count(segment_t const count) {
        segment_count = count;
        // For now
        if (segment_idx >= segment_count)
            segment_idx = segment_count - 1;
    }
    // Time to wait for servo to move to position before starting measurement
    static void set_movement_time(tick_t time_ms) { movement_time_ms = time_ms; }

    static void start() {
         HCSR04::falling_callback_fn = [](tick_t const pulse_length) {
            pulse_length_buf[buf_idx++] = pulse_length;
            // Received all pulses for this position
            if (buf_idx == avg_buf_len) {
                buf_idx = 0;
                // Calculate average pulse length
                tick_t avg_pulse_length = 0;
                for (uint8_t i = 0; i < avg_buf_len; i++)
                    avg_pulse_length += pulse_length_buf[i];
                avg_pulse_length /= avg_buf_len;

                // Use a temporary non-volatile variable for assignment
                most_recent_measurement.deg = inprogress_measurement_deg;
                most_recent_measurement.pulse_length = avg_pulse_length;

                // Set new servo position
                segment_idx += direction;
                if (segment_idx == segment_count || segment_idx == 0)
                    direction = -direction;
                inprogress_measurement_deg = segment_idx * (deg_max - deg_min) / segment_count + deg_min;
                servo::set_position(inprogress_measurement_deg);
                // Wait until in position before staring measurement
                thread_flags.schedule_next_position = true;
                timer.in(movement_time_ms * 1000, [](auto) {
                    HCSR04::send_pulse();
                    return false;
                });
            }
            else {
                thread_flags.send_pulse = true;
            }
        };
        thread_flags.send_pulse = true;
    }
    static void stop() {
        HCSR04::falling_callback_fn = nullptr;
    }
    static void update() {
        if (thread_flags.send_pulse) {
            thread_flags.send_pulse = false;
            // Recommends 60ms measurement interval in datasheet
            timer.in(10UL * 1000UL, [](auto) {
                    HCSR04::send_pulse();
                    return false;
            });
        }
    }
};

template <uint8_t avg_buf_len>
deg_t RangeFinder<avg_buf_len>::deg_min = 0;
template <uint8_t avg_buf_len>
deg_t RangeFinder<avg_buf_len>::deg_max = 0;
template <uint8_t avg_buf_len>
deg_t RangeFinder<avg_buf_len>::inprogress_measurement_deg = 0;
template <uint8_t avg_buf_len>
typename RangeFinder<avg_buf_len>::segment_t RangeFinder<avg_buf_len>::segment_count = 0;
template <uint8_t avg_buf_len>
typename RangeFinder<avg_buf_len>::segment_t RangeFinder<avg_buf_len>::segment_idx = 0;
template <uint8_t avg_buf_len>
int8_t RangeFinder<avg_buf_len>::direction = 1;
template <uint8_t avg_buf_len>
tick_t RangeFinder<avg_buf_len>::movement_time_ms = 0;
template <uint8_t avg_buf_len>
volatile tick_t RangeFinder<avg_buf_len>::pulse_length_buf[avg_buf_len];
template <uint8_t avg_buf_len>
volatile uint8_t RangeFinder<avg_buf_len>::buf_idx = 0;
template <uint8_t avg_buf_len>
volatile typename RangeFinder<avg_buf_len>::Measurement RangeFinder<avg_buf_len>::most_recent_measurement;
template <uint8_t avg_buf_len>
volatile typename RangeFinder<avg_buf_len>::ThreadFlags RangeFinder<avg_buf_len>::thread_flags;

using RangeFinder_t = RangeFinder<4>;

void setup() {
    // debug_init();
    Serial.begin(9600);
    delay(1000);
    servo::setup();
    HCSR04::setup();

    RangeFinder_t::set_degree_range(0, 180);
    RangeFinder_t::set_segment_count(10);
    RangeFinder_t::set_movement_time(100);
    RangeFinder_t::start();
}

void loop() {
    timer.tick();

    if (reached_pos) {
        Serial.println("At pos");
        reached_pos = 0;
    }
    RangeFinder_t::update();

    // delay(50);
    // auto v = (static_cast<uint32_t>(analogRead(A0)) * 180) / 1024;
    // servo::set_position(v);
}
