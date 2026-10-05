/* uav-pps-trigger: PPS-disciplined 60 Hz trigger train for the Hadron 640R+ one-shot slave sync.
 * Design: docs/hadron-thermal.md, "Timestamping: one-shot slave sync from a PPS-disciplined MCU".
 *
 * Pins (Waveshare RP2040-Zero)
 *   GPIO2  PPS in    rising edge, 3.3 V from the receiver's PPS fan-out (weak pull-down when unplugged)
 *   GPIO3  TRIG out  the 60 Hz train, 100 us high; 100 R in series to the r2 board's J3-1 (VSYNC_3V3), J3-2 GND
 *   GPIO4  EVT in    spare event input (chopper slot sensor), rising edge, reported as $EVT
 *   GPIO5  TEST out  1 Hz, 5 ms test pulse while "TEST 1", high impedance otherwise; jumper to GPIO2 for a self-test
 *   GPIO16 WS2812    red: no PPS yet, amber: acquiring, green: locked, magenta: hold-over; blue blink per PPS edge
 *
 * Time bases. The train comes from a PIO state machine with intervals in system-clock
 * cycles (125 MHz, 8 ns), fed from the TRIG edge interrupt, so a busy main loop can
 * never starve it. PPS edges, the train's own edges and events are timestamped by the
 * 1 MHz hardware timer in the GPIO interrupt. Both clocks derive from the one 12 MHz
 * crystal, so they never drift against each other.
 *
 * Control. Each PPS interval measures the crystal against GPS time; the median of the
 * last five is the period P, and a second's 60 intervals are P/60 with the remainder
 * spread Bresenham-style. Every pulse is labelled on the PPS grid (second N, index k,
 * pulse 0 on the edge) and its offset from the grid is measured. The queue is four
 * pulses deep, so a correction is seen four pulses later: the controller subtracts the
 * corrections already queued from the measured offset (a dead-time predictor; without
 * it the loop limit-cycles at +-4 x the bound, measured +-200 us) and asks the next
 * interval for the remainder, bounded to 0.3 % so the instantaneous rate stays inside
 * the camera's 59.75-60.25 Hz window, never stepped. Without PPS the last
 * period is held (status H), or the crystal's nominal 60 Hz before the first edge
 * (status N). A pull on an empty FIFO only ever makes a pulse late (trigger.pio).
 *
 * Reports (NMEA-style: $body*XX, XOR of the body; only while a host has the port open)
 *   $HELLO,boot,fw,sysclk_hz,pps_pin,trig_pin,evt_pin,test_pin,test_on          at connect, every 10 s
 *   $PPS,boot,N,period_us,ppm,offset_us,status,missed                              per PPS edge
 *   $TRG,boot,seq,N,k,offset_ns,status                                            per pulse (60/s)
 *   $EVT,boot,N,us_since_pps                                                       per event edge
 *   $STAT,boot,status,pulses,underruns,pps_edges,glitches,bad_periods,missed,max_abs_offset_us   every 10 s
 * Commands, one per line: TEST 1 | TEST 0 | TESTW <ms> (test pulse width, default 5) | STATUS | PINS | RESET
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "pico/rand.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "trigger.pio.h"
#include "ws2812.pio.h"

#define FW_VERSION "0.1.0"
#define PIN_PPS 2
#define PIN_TRIG 3
#define PIN_EVT 4
#define PIN_TEST 5
#define PIN_LED 16

#define PULSES_PER_S 60u
#define PULSE_HIGH_US 100u
#define RATE_MIN_HZ 59.75         /* the Boson's validated external-sync window */
#define RATE_MAX_HZ 60.25
#define MAX_SLEW_PPM 3000         /* per-interval phase correction bound: 0.3 % of P/60 (~50 us) */
#define PPS_PPM_LIMIT 500.0       /* an interval outside 1 s +- 500 ppm is not a PPS second */
#define PPS_DEBOUNCE_US 900000.0
#define LOCK_US 50.0              /* |offset| below this on 60 consecutive pulses: locked */
#define UNLOCK_US 500.0
#define HOLDOVER_US 1300000ull
#define TEST_PULSE_US 5000          /* default; TESTW <ms> changes it (bench checks with a meter) */
#define RING 64

static PIO pio = pio0;
static uint sm_trig, sm_led;
static uint32_t cycles_per_us, high_cycles, interval_min, interval_max;
static uint32_t boot_id;

/* ---- shared with the interrupt handler ---- */
static volatile uint64_t trig_ring[RING], pps_ring[16], evt_ring[16];
static volatile uint32_t trig_head, pps_head, evt_head;
static volatile uint32_t period_cycles;      /* P in cycles, set by the main loop */
static volatile int32_t pending_slew;        /* cycles still to apply, consumed per interval */
static volatile int32_t max_slew;
static volatile uint32_t underruns, pulses_queued, pps_irqs, test_pulses;
static volatile int32_t slew_applied[RING];   /* correction built into the interval ending at queued pulse q, by q % RING */
static uint32_t bres_acc;                    /* interrupt context only */

static void queue_pulse(void) {
    uint32_t p = period_cycles;
    uint32_t interval = p / PULSES_PER_S;
    bres_acc += p % PULSES_PER_S;
    if (bres_acc >= PULSES_PER_S) { bres_acc -= PULSES_PER_S; interval++; }
    int32_t slew = pending_slew, bound = max_slew;
    if (slew > bound) slew = bound; else if (slew < -bound) slew = -bound;
    pending_slew -= slew;
    uint32_t q = pulses_queued + 1;
    slew_applied[q % RING] = slew;
    interval = (uint32_t)((int32_t)interval + slew);
    if (interval < interval_min) interval = interval_min;   /* never faster than 60.25 Hz */
    if (interval > interval_max) interval = interval_max;   /* never slower than 59.75 Hz */
    pio_sm_put(pio, sm_trig, interval - high_cycles - 5);   /* x: low cycles (interval = x + y + 7) */
    pio_sm_put(pio, sm_trig, high_cycles - 2);              /* y: high cycles (high = y + 2) */
    pulses_queued = q;
}

static void on_gpio(uint gpio, uint32_t events) {
    (void)events;
    uint64_t now = time_us_64();
    if (gpio == PIN_TRIG) {
        trig_ring[trig_head % RING] = now;
        trig_head++;
        if (pio_sm_get_tx_fifo_level(pio, sm_trig) < 2) underruns++;   /* the queue ran dry */
        if (!pio_sm_is_tx_fifo_full(pio, sm_trig)) queue_pulse();      /* one pulse consumed, one queued: four ahead */
    } else if (gpio == PIN_PPS) {
        pps_ring[pps_head % 16] = now;
        pps_head++;
        pps_irqs++;
    } else if (gpio == PIN_EVT) {
        evt_ring[evt_head % 16] = now;
        evt_head++;
    }
}

/* ---- test PPS (1 Hz, 5 ms) from the same crystal ---- */
static repeating_timer_t test_timer;
static bool test_on;
static volatile uint32_t test_width_us = TEST_PULSE_US;
static int64_t test_low(alarm_id_t id, void *user) { (void)id; (void)user; gpio_put(PIN_TEST, 0); return 0; }
static bool test_high(repeating_timer_t *t) { (void)t; gpio_put(PIN_TEST, 1); test_pulses++; add_alarm_in_us(test_width_us, test_low, NULL, true); return true; }
/* GPIO5 drives only while the self-test is on. Idle it is an input (high impedance):
 * a bench unit may have GPIO5 soldered to GPIO2, and the flight unit's GPIO2 is on
 * the receiver's passive PPS net, which nothing but the receiver may ever drive. */
static void set_test(bool on) {
    if (on == test_on) return;
    if (on) { gpio_put(PIN_TEST, 0); gpio_set_dir(PIN_TEST, GPIO_OUT); add_repeating_timer_us(-1000000, test_high, NULL, &test_timer); }
    else { cancel_repeating_timer(&test_timer); gpio_put(PIN_TEST, 0); gpio_set_dir(PIN_TEST, GPIO_IN); }
    test_on = on;
}

/* ---- reports ---- */
static void report(const char *fmt, ...) {
    if (!stdio_usb_connected()) return;
    char body[160];
    va_list ap; va_start(ap, fmt); vsnprintf(body, sizeof body, fmt, ap); va_end(ap);
    uint8_t cs = 0;
    for (const char *c = body; *c; c++) cs ^= (uint8_t)*c;
    printf("$%s*%02X\r\n", body, cs);
}

static void put_pixel(uint32_t grb) {
    if (!pio_sm_is_tx_fifo_full(pio, sm_led)) pio_sm_put(pio, sm_led, grb << 8);
}

static double median5(double *v, int n) {
    double s[5]; memcpy(s, v, n * sizeof(double));
    for (int i = 1; i < n; i++) for (int j = i; j > 0 && s[j - 1] > s[j]; j--) { double t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
    return n % 2 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
}

int main(void) {
    stdio_init_all();
    cycles_per_us = clock_get_hz(clk_sys) / 1000000u;
    high_cycles = PULSE_HIGH_US * cycles_per_us;
    period_cycles = 1000000u * cycles_per_us;
    interval_min = (uint32_t)(cycles_per_us * 1000000.0 / RATE_MAX_HZ);
    interval_max = (uint32_t)(cycles_per_us * 1000000.0 / RATE_MIN_HZ);
    max_slew = (int32_t)((uint64_t)period_cycles / PULSES_PER_S * MAX_SLEW_PPM / 1000000u);
    boot_id = get_rand_32();

    gpio_init(PIN_PPS); gpio_set_dir(PIN_PPS, GPIO_IN); gpio_pull_down(PIN_PPS);
    gpio_init(PIN_EVT); gpio_set_dir(PIN_EVT, GPIO_IN); gpio_pull_down(PIN_EVT);
    gpio_init(PIN_TEST); gpio_set_dir(PIN_TEST, GPIO_IN); gpio_disable_pulls(PIN_TEST);   /* high impedance until TEST 1 */

    uint off = pio_add_program(pio, &trigger_program);
    sm_trig = pio_claim_unused_sm(pio, true);
    trigger_program_init(pio, sm_trig, off, PIN_TRIG);
    uint off_led = pio_add_program(pio, &ws2812_program);
    sm_led = pio_claim_unused_sm(pio, true);
    ws2812_program_init(pio, sm_led, off_led, PIN_LED, 800000.0f);

    for (int i = 0; i < 4; i++) queue_pulse();                      /* four pulses ahead, then the interrupt tops up */
    gpio_set_irq_enabled_with_callback(PIN_TRIG, GPIO_IRQ_EDGE_RISE, true, &on_gpio);
    gpio_set_irq_enabled(PIN_PPS, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(PIN_EVT, GPIO_IRQ_EDGE_RISE, true);
    pio_sm_set_enabled(pio, sm_trig, true);
    irq_set_priority(IO_IRQ_BANK0, 0);          /* edge timestamps ahead of USB and timer interrupts */
    watchdog_enable(3000, 1);

    /* main-loop state */
    char status = 'N';
    uint64_t last_pps_us = 0;        /* time of the last real PPS edge */
    uint32_t pps_N = 0;              /* PPS count at that edge (first edge = 1) */
    double P_us = 1000000.0, periods[5]; int period_n = 0, period_i = 0;
    uint32_t trig_tail = 0, pps_tail = 0, evt_tail = 0, seq = 0;
    uint32_t glitches = 0, bad_periods = 0, missed = 0, pps_edges = 0;
    double max_abs_offset = 0, last_offset_us = 0;
    int lock_run = 0;
    uint64_t next_hello = 0, next_stat = 10000000, led_flash_until = 0, last_led = 0;
    bool was_connected = false;
    char cmd[32]; int cmd_n = 0;

    while (true) {
        watchdog_update();
        uint64_t now = time_us_64();

        /* PPS edges */
        while (pps_tail != pps_head) {
            uint64_t t = pps_ring[pps_tail % 16]; pps_tail++;
            pps_edges++;
            led_flash_until = now + 60000;
            if (last_pps_us == 0) {
                last_pps_us = t; pps_N = 1; status = 'A'; lock_run = 0;
                report("PPS,%08lX,%lu,0,0.0,%.1f,%c,0", (unsigned long)boot_id, (unsigned long)pps_N, last_offset_us, status);
                continue;
            }
            double dt = (double)(t - last_pps_us);
            if (dt < PPS_DEBOUNCE_US) { glitches++; continue; }
            int m = (int)llround(dt / P_us); if (m < 1) m = 1;
            double est = dt / m;
            if (fabs(est - 1000000.0) > PPS_PPM_LIMIT) {
                bad_periods++;                                      /* an edge we will not learn the period from */
            } else {
                periods[period_i++ % 5] = est; if (period_n < 5) period_n++;
                P_us = median5(periods, period_n);
                period_cycles = (uint32_t)llround(P_us * cycles_per_us);
            }
            missed += (uint32_t)(m - 1);
            pps_N += (uint32_t)m;
            last_pps_us = t;
            if (status == 'H' || status == 'N') { status = 'A'; lock_run = 0; }
            report("PPS,%08lX,%lu,%.1f,%+.1f,%+.1f,%c,%lu", (unsigned long)boot_id, (unsigned long)pps_N, est, est - 1000000.0,
                   last_offset_us, status, (unsigned long)missed);
        }

        /* hold-over */
        if (last_pps_us && now - last_pps_us > HOLDOVER_US && status != 'H') { status = 'H'; lock_run = 0; }

        /* the train's own edges: label on the PPS grid, measure, correct, report */
        while (trig_tail != trig_head) {
            uint64_t t = trig_ring[trig_tail % RING]; trig_tail++;
            seq++;
            if (last_pps_us == 0) {
                report("TRG,%08lX,%lu,0,%lu,0,%c", (unsigned long)boot_id, (unsigned long)seq, (unsigned long)(seq % PULSES_PER_S), status);
                continue;
            }
            double rel = (double)((int64_t)t - (int64_t)last_pps_us);
            double slot = P_us / PULSES_PER_S;
            long long m = (long long)floor((rel + slot / 2) / P_us);
            double in_sec = rel - (double)m * P_us;
            int k = (int)lround(in_sec / slot);
            if (k >= (int)PULSES_PER_S) { k = 0; m++; }
            double offset_us = rel - ((double)m * P_us + k * slot);
            last_offset_us = offset_us;
            if (fabs(offset_us) > max_abs_offset) max_abs_offset = fabs(offset_us);
            /* corrections already built into the pulses still queued after this one */
            uint32_t queued = pulses_queued;
            int64_t inflight = 0;
            for (uint32_t j = seq + 1; j <= queued && j < seq + RING; j++) inflight += slew_applied[j % RING];
            pending_slew = (int32_t)(llround(-offset_us * cycles_per_us) - inflight);   /* replace, never accumulate */
            if (fabs(offset_us) < LOCK_US) { if (lock_run < 1000) lock_run++; } else lock_run = 0;
            if (status == 'A' && lock_run >= (int)PULSES_PER_S) status = 'L';
            if (status == 'L' && fabs(offset_us) > UNLOCK_US) { status = 'A'; lock_run = 0; }
            report("TRG,%08lX,%lu,%lu,%d,%lld,%c", (unsigned long)boot_id, (unsigned long)seq,
                   (unsigned long)(pps_N + m), k, (long long)llround(offset_us * 1000.0), status);
        }

        /* events */
        while (evt_tail != evt_head) {
            uint64_t t = evt_ring[evt_tail % 16]; evt_tail++;
            if (last_pps_us == 0) { report("EVT,%08lX,0,%llu", (unsigned long)boot_id, (unsigned long long)t); continue; }
            double rel = (double)((int64_t)t - (int64_t)last_pps_us);
            long long m = (long long)floor(rel / P_us);
            report("EVT,%08lX,%lu,%.0f", (unsigned long)boot_id, (unsigned long)(pps_N + m), rel - (double)m * P_us);
        }

        /* host side: hello on connect and every 10 s, stats every 10 s, commands */
        bool connected = stdio_usb_connected();
        if ((connected && !was_connected) || (connected && now >= next_hello)) {
            report("HELLO,%08lX,%s,%lu,%d,%d,%d,%d,%d", (unsigned long)boot_id, FW_VERSION, (unsigned long)clock_get_hz(clk_sys),
                   PIN_PPS, PIN_TRIG, PIN_EVT, PIN_TEST, test_on ? 1 : 0);
            next_hello = now + 10000000;
        }
        was_connected = connected;
        if (now >= next_stat) {
            report("STAT,%08lX,%c,%lu,%lu,%lu,%lu,%lu,%lu,%.1f", (unsigned long)boot_id, status, (unsigned long)seq,
                   (unsigned long)underruns, (unsigned long)pps_edges, (unsigned long)glitches, (unsigned long)bad_periods,
                   (unsigned long)missed, max_abs_offset);
            next_stat = now + 10000000;
        }
        int ch;
        while ((ch = getchar_timeout_us(0)) >= 0) {
            if (ch == '\n' || ch == '\r') {
                cmd[cmd_n] = 0;
                if (!strcmp(cmd, "TEST 1")) set_test(true);
                else if (!strcmp(cmd, "TEST 0")) set_test(false);
                else if (!strcmp(cmd, "STATUS")) { next_hello = 0; next_stat = 0; }
                else if (!strcmp(cmd, "RESET")) watchdog_reboot(0, 0, 0);
                else if (!strncmp(cmd, "TESTW ", 6)) { long w = atol(cmd + 6); if (w >= 1 && w <= 900) test_width_us = (uint32_t)w * 1000u; report("TESTW,%lu", (unsigned long)test_width_us / 1000u); }
                else if (!strcmp(cmd, "PINS")) report("PINS,pps=%d,evt=%d,test=%d,test_on=%d,test_pulses=%lu,pps_irqs=%lu",
                                                      gpio_get(PIN_PPS), gpio_get(PIN_EVT), gpio_get(PIN_TEST), test_on ? 1 : 0,
                                                      (unsigned long)test_pulses, (unsigned long)pps_irqs);
                else if (cmd_n) report("ERR,unknown command %s", cmd);
                cmd_n = 0;
            } else if (cmd_n < (int)sizeof cmd - 1) {
                cmd[cmd_n++] = (char)ch;
            }
        }

        /* LED at 20 Hz */
        if (now - last_led > 50000) {
            last_led = now;
            uint32_t grb = status == 'L' ? 0x100000 : status == 'A' ? 0x081000 : status == 'H' ? 0x001010 : 0x001000;
            if (now < led_flash_until) grb = 0x000018;
            put_pixel(grb);
        }
        sleep_us(200);
    }
}
