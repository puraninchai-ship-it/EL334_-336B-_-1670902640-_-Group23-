#include "MK64F12.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
// FRDM-K64F timer with Multi-Function Shield and I2C LCD.
// S1 selects a Countdown digit. S2 increments it. S3 starts, pauses or resumes.
// SW2 changes mode. SW3 requests a reset through the PORTA interrupt.

// -----------------------------------------------------------------------------
// Board pin assignments
// -----------------------------------------------------------------------------

// Multi-Function Shield: 74HC595 on Arduino D8, D7, D4.
#define DATA_PIN        12U           // PTC12
#define CLOCK_PIN        3U           // PTC3
#define LATCH_PIN       23U           // PTB23
#define DATA_MASK       (1UL << DATA_PIN)
#define CLOCK_MASK      (1UL << CLOCK_PIN)
#define LATCH_MASK      (1UL << LATCH_PIN)
// Buttons
#define S1_PIN           3U           // PTB3, Arduino A1
#define S2_PIN          10U           // PTB10, Arduino A2
#define S3_PIN          11U           // PTB11, Arduino A3
#define SW2_PIN          6U           // PTC6
#define SW3_PIN          4U           // PTA4
#define SW3_MASK        (1UL << SW3_PIN)
#define SW3_FALLING_IRQ  0xAU
// Alarm outputs
#define BUZZER_PIN       1U           // PTA1, Arduino D3, active low
#define BUZZER_MASK     (1UL << BUZZER_PIN)
#define ALARM_LED_PIN   17U           // External LED: PTC17, active high
#define ALARM_LED_MASK  (1UL << ALARM_LED_PIN)
#define SHIELD_LED_PIN   1U           // Shield LED D1: PTD1, active low
#define SHIELD_LED_MASK (1UL << SHIELD_LED_PIN)
// The shield's APC220 socket routes D1/PTC17 (pin 4) and D0/PTC16 (pin 5).
// Select 2 only when LCD SDA is wired separately to D6/PTC2.
#ifndef LCD_SDA_PIN
#define LCD_SDA_PIN     17U
#endif
#define LCD_SCL_PIN     16U           // PTC16, Arduino D0
#define LCD_SDA_MASK    (1UL << LCD_SDA_PIN)
#define LCD_SCL_MASK    (1UL << LCD_SCL_PIN)

// -----------------------------------------------------------------------------
// Shared timer state
// -----------------------------------------------------------------------------

#define SECOND_MS    1000U
#define MAX_SECONDS  5999U  // 99:59
typedef enum
{
    COUNTDOWN,
    STOPWATCH
} Mode;
typedef enum
{
    IDLE,
    RUNNING,
    PAUSED
} TimerState;
static Mode mode = COUNTDOWN;
static TimerState timer_state = IDLE;
static uint32_t ms_ticks;         // Elapsed time used by all timed subsystems.
static uint32_t seconds;          // Time shown in the active mode.
static uint32_t saved_countdown;  // Countdown time held while in Stopwatch.
static uint32_t last_second_tick;
static uint32_t paused_ms;
static uint8_t selected_digit;
static bool blink_enabled;

// -----------------------------------------------------------------------------
// GPIO setup
// -----------------------------------------------------------------------------

static void GPIO_Init(void)
{
    const uint32_t button_config =
        PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    const uint32_t shield_buttons =
        (1UL << S1_PIN) | (1UL << S2_PIN) | (1UL << S3_PIN);
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK |
                  SIM_SCGC5_PORTB_MASK |
                  SIM_SCGC5_PORTC_MASK |
                  SIM_SCGC5_PORTD_MASK;
    PORTC->PCR[DATA_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[CLOCK_PIN] = PORT_PCR_MUX(1);
    PORTB->PCR[LATCH_PIN] = PORT_PCR_MUX(1);
    PTC->PCOR = DATA_MASK | CLOCK_MASK;
    PTB->PCOR = LATCH_MASK;
    PTC->PDDR |= DATA_MASK | CLOCK_MASK;
    PTB->PDDR |= LATCH_MASK;
    PORTB->PCR[S1_PIN] = button_config;
    PORTB->PCR[S2_PIN] = button_config;
    PORTB->PCR[S3_PIN] = button_config;
    PTB->PDDR &= ~shield_buttons;
    PORTC->PCR[SW2_PIN] = button_config;
    PTC->PDDR &= ~(1UL << SW2_PIN);
    PORTA->PCR[SW3_PIN] = button_config;
    PTA->PDDR &= ~SW3_MASK;
    // Release D3 while the buzzer is silent. Its output latch stays low.
    PTA->PDDR &= ~BUZZER_MASK;
    PTA->PCOR = BUZZER_MASK;
    PORTA->PCR[BUZZER_PIN] = PORT_PCR_MUX(1);
#if LCD_SDA_PIN != ALARM_LED_PIN
    PTC->PCOR = ALARM_LED_MASK;
    PORTC->PCR[ALARM_LED_PIN] = PORT_PCR_MUX(1);
    PTC->PDDR |= ALARM_LED_MASK;
#endif
    // Sink current to light the shield LED. Release the pin to turn it off.
    PTD->PDDR &= ~SHIELD_LED_MASK;
    PTD->PCOR = SHIELD_LED_MASK;
    PORTD->PCR[SHIELD_LED_PIN] = PORT_PCR_MUX(1);
    // Use inputs to release the bus. A pin only becomes an output to drive low.
    PTC->PDDR &= ~(LCD_SDA_MASK | LCD_SCL_MASK);
    PTC->PCOR = LCD_SDA_MASK | LCD_SCL_MASK;
    PORTC->PCR[LCD_SDA_PIN] = PORT_PCR_MUX(1);
    PORTC->PCR[LCD_SCL_PIN] = PORT_PCR_MUX(1);
}

// -----------------------------------------------------------------------------
// Four-digit display and digit selection indicator
// -----------------------------------------------------------------------------

#define BLINK_PHASE_MS      500U
#define BLINK_DIGIT_SHIFT    16U
#define SEGMENTS_OFF       0xFFU
static const uint8_t segment_map[10] =
{
    0xC0U, 0xF9U, 0xA4U, 0xB0U, 0x99U,
    0x92U, 0x82U, 0xF8U, 0x80U, 0x90U
};

static const uint8_t digit_select[4] = {0xF1U, 0xF2U, 0xF4U, 0xF8U};
// The low 16 bits hold the four BCD digits.
// The upper bits hold the blink digit (1 to 4), or 0 for no blinking.
static volatile uint32_t display_frame;
static void Shift_Delay(void)
{
    __NOP();
    __NOP();
    __NOP();
    __NOP();
}

static void Shift_Byte(uint8_t value)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        if ((value & mask) != 0U)
        {
            PTC->PSOR = DATA_MASK;
        }
        else
        {
            PTC->PCOR = DATA_MASK;
        }

        Shift_Delay();
        PTC->PSOR = CLOCK_MASK;
        Shift_Delay();
        PTC->PCOR = CLOCK_MASK;
    }
}

static void Display_Scan(void)
{
    static uint8_t digit;
    static uint32_t frame;
    static uint32_t blink_digit;
    static uint32_t phase_started_at;
    static bool blink_hidden;
    if (digit == 0U)
    {
        frame = display_frame;
        const uint32_t next_blink_digit = frame >> BLINK_DIGIT_SHIFT;
        const uint32_t now = ms_ticks;
        if (next_blink_digit != blink_digit)
        {
            blink_digit = next_blink_digit;
            phase_started_at = now;
            blink_hidden = false;
        }
        else if (blink_digit != 0U &&
                 (now - phase_started_at) >= BLINK_PHASE_MS)
        {
            phase_started_at += BLINK_PHASE_MS;
            blink_hidden = !blink_hidden;
        }
    }

    const uint32_t value = (frame >> (12U - 4U * digit)) & 0x0FU;
    const uint8_t segments = (blink_hidden && blink_digit == digit + 1U) ?
                             SEGMENTS_OFF : segment_map[value];
    // Send the segment pattern first, then the digit-select byte.
    Shift_Byte(segments);
    Shift_Byte(digit_select[digit]);
    Shift_Delay();
    PTB->PSOR = LATCH_MASK;
    Shift_Delay();
    PTB->PCOR = LATCH_MASK;
    digit = (uint8_t)((digit + 1U) % 4U);
}

static void Display_Time(void)
{
    const uint32_t minutes = seconds / 60U;
    const uint32_t remainder = seconds % 60U;
    const uint32_t blink_digit = blink_enabled ? selected_digit + 1U : 0U;
    display_frame = (blink_digit << BLINK_DIGIT_SHIFT) |
                    ((minutes / 10U) << 12U) |
                    ((minutes % 10U) << 8U) |
                    ((remainder / 10U) << 4U) |
                    (remainder % 10U);
}

// -----------------------------------------------------------------------------
// SysTick timebase using polling
// -----------------------------------------------------------------------------

static uint32_t cycles_per_ms;
static uint32_t clock_previous;
static uint32_t clock_remainder;
static void Clock_Init(void)
{
    cycles_per_ms = SystemCoreClock / SECOND_MS;
    SysTick->CTRL = 0U;
    SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
    SysTick->VAL = 0U;
    // Leave TICKINT clear: the main loop reads the hardware counter.
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    clock_previous = SysTick->VAL;
}

static void Clock_Update(void)
{
    const uint32_t current = SysTick->VAL;
    // The 24-bit downcounter runs continuously, including during LCD delays.
    clock_remainder += (clock_previous - current) & SysTick_LOAD_RELOAD_Msk;
    clock_previous = current;
    if (clock_remainder >= cycles_per_ms)
    {
        ms_ticks += clock_remainder / cycles_per_ms;
        clock_remainder %= cycles_per_ms;
        Display_Scan();
    }
}

static void Delay_Ms(uint32_t delay)
{
    const uint32_t start = ms_ticks;
    while ((ms_ticks - start) < delay)
    {
        __NOP();
        Clock_Update();
    }
}

// -----------------------------------------------------------------------------
// I2C bus and LCD mode display
// -----------------------------------------------------------------------------

#define LCD_ADDR_PCF8574  0x27U
#define LCD_ADDR_PCF8574A 0x3FU
#define LCD_RETRY_MS      1000U
#define I2C_DELAY_US         5U
#define I2C_CLOCK_TRIES     20U
#define LCD_RS          0x01U
#define LCD_ENABLE      0x04U
#define LCD_BACKLIGHT   0x08U
static uint32_t i2c_delay_cycles;
static uint8_t lcd_address;
static void I2C_Delay(void)
{
    // This short wait uses the same free-running counter as the timebase.
    const uint32_t period = SysTick->LOAD + 1U;
    const uint32_t start = SysTick->VAL;
    uint32_t elapsed;
    do
    {
        __NOP();
        const uint32_t now = SysTick->VAL;
        elapsed = (start >= now) ? start - now : start + period - now;
    } while (elapsed < i2c_delay_cycles);
    Clock_Update();
}

static bool I2C_RaiseClock(void)
{
    PTC->PDDR &= ~LCD_SCL_MASK;
    for (uint8_t attempt = 0U; attempt < I2C_CLOCK_TRIES; attempt++)
    {
        I2C_Delay();
        if ((PTC->PDIR & LCD_SCL_MASK) != 0U)
        {
            return true;
        }
    }

    return false;
}

static bool I2C_Start(void)
{
    PTC->PDDR &= ~LCD_SDA_MASK;
    I2C_Delay();
    if (!I2C_RaiseClock() || (PTC->PDIR & LCD_SDA_MASK) == 0U)
    {
        return false;
    }

    PTC->PDDR |= LCD_SDA_MASK;
    I2C_Delay();
    PTC->PDDR |= LCD_SCL_MASK;
    I2C_Delay();
    return true;
}

static bool I2C_Stop(void)
{
    PTC->PDDR |= LCD_SDA_MASK;
    I2C_Delay();
    const bool clock_high = I2C_RaiseClock();
    PTC->PDDR &= ~LCD_SDA_MASK;
    I2C_Delay();
    return clock_high && (PTC->PDIR & LCD_SDA_MASK) != 0U;
}

static bool I2C_Clock(bool check_ack)
{
    I2C_Delay();
    if (!I2C_RaiseClock())
    {
        PTC->PDDR |= LCD_SCL_MASK;
        I2C_Delay();
        return false;
    }

    const bool accepted = !check_ack || (PTC->PDIR & LCD_SDA_MASK) == 0U;
    PTC->PDDR |= LCD_SCL_MASK;
    I2C_Delay();
    return accepted;
}

static bool I2C_Write(uint8_t value)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
    {
        if ((value & mask) != 0U)
        {
            PTC->PDDR &= ~LCD_SDA_MASK;
        }
        else
        {
            PTC->PDDR |= LCD_SDA_MASK;
        }

        if (!I2C_Clock(false))
        {
            return false;
        }
    }

    PTC->PDDR &= ~LCD_SDA_MASK;
    return I2C_Clock(true);
}

static bool I2C_RecoverBus(void)
{
    PTC->PDDR &= ~(LCD_SDA_MASK | LCD_SCL_MASK);
    if (!I2C_RaiseClock())
    {
        return false;
    }

    if ((PTC->PDIR & LCD_SDA_MASK) != 0U)
    {
        return true;
    }

    // A reset during a transfer can leave the slave driving SDA.
    for (uint8_t pulse = 0U; pulse < 9U; pulse++)
    {
        PTC->PDDR |= LCD_SCL_MASK;
        I2C_Delay();
        if (!I2C_RaiseClock())
        {
            return false;
        }

        if ((PTC->PDIR & LCD_SDA_MASK) != 0U)
        {
            PTC->PDDR |= LCD_SCL_MASK;
            I2C_Delay();
            return I2C_Stop();
        }
    }

    return false;
}

static bool PCF_Write(uint8_t address, const uint8_t *values, size_t count)
{
    if (!I2C_Start())
    {
        return false;
    }

    bool accepted = I2C_Write((uint8_t)(address << 1U));
    for (size_t i = 0U; accepted && i < count; i++)
    {
        accepted = I2C_Write(values[i]);
    }

    const bool stopped = I2C_Stop();
    return accepted && stopped;
}

static bool LCD_Write4(uint8_t value)
{
    value |= LCD_BACKLIGHT;
    const uint8_t pulse[] = {value, value | LCD_ENABLE, value};
    return PCF_Write(lcd_address, pulse, sizeof pulse);
}

static bool LCD_Send(uint8_t value, uint8_t rs)
{
    return LCD_Write4((value & 0xF0U) | rs) &&
           LCD_Write4((uint8_t)((value << 4U) & 0xF0U) | rs);
}

static uint8_t LCD_FindAddress(void)
{
    // Probe the PCF8574 and PCF8574A address ranges without writing data.
    static const uint8_t addresses[] =
    {
        LCD_ADDR_PCF8574, LCD_ADDR_PCF8574A,
        0x20U, 0x21U, 0x22U, 0x23U, 0x24U, 0x25U,
        0x26U, 0x38U, 0x39U, 0x3AU, 0x3BU, 0x3CU, 0x3DU, 0x3EU
    };

    if (!I2C_RecoverBus())
    {
        return 0U;
    }

    for (size_t i = 0U; i < sizeof addresses; i++)
    {
        if (PCF_Write(addresses[i], NULL, 0U))
        {
            return addresses[i];
        }
    }

    return 0U;
}

static bool LCD_Init(void)
{
    const uint8_t idle = LCD_BACKLIGHT;
    lcd_address = LCD_FindAddress();
    if (lcd_address == 0U || !PCF_Write(lcd_address, &idle, 1U))
    {
        return false;
    }

    if (!LCD_Write4(0x30U))
    {
        return false;
    }

    Delay_Ms(6U);
    if (!LCD_Write4(0x30U))
    {
        return false;
    }

    Delay_Ms(2U);
    if (!LCD_Write4(0x30U))
    {
        return false;
    }

    Delay_Ms(2U);
    if (!LCD_Write4(0x20U))
    {
        return false;
    }

    Delay_Ms(2U);
    if (!LCD_Send(0x28U, 0U) || !LCD_Send(0x08U, 0U) ||
        !LCD_Send(0x01U, 0U))
    {
        return false;
    }

    Delay_Ms(3U);
    return LCD_Send(0x06U, 0U) && LCD_Send(0x0CU, 0U);
}

static bool LCD_ShowMode(void)
{
    const char *text = (mode == COUNTDOWN) ? "Mode: COUNTDOWN " :
                                            "Mode: STOPWATCH ";
    if (!LCD_Send(0x80U, 0U))
    {
        return false;
    }

    while (*text != '\0')
    {
        if (!LCD_Send((uint8_t)*text, LCD_RS))
        {
            return false;
        }

        text++;
    }

    return true;
}

static bool LCD_Update(uint32_t now)
{
    static bool ready;
    static Mode shown_mode;
    static uint32_t last_attempt = 0U - LCD_RETRY_MS;
    if (!ready)
    {
        if ((now - last_attempt) < LCD_RETRY_MS)
        {
            return false;
        }

        last_attempt = now;
        ready = LCD_Init() && LCD_ShowMode();
        if (ready)
        {
            shown_mode = mode;
        }
    }
    else if (mode != shown_mode)
    {
        ready = LCD_ShowMode();
        shown_mode = mode;
        if (!ready)
        {
            last_attempt = now;
        }
    }

    return ready;
}

// -----------------------------------------------------------------------------
// Buzzer and alarm LEDs
// -----------------------------------------------------------------------------

#define BUZZER_FINISH_BEEPS   10U
#define BUZZER_ON_MS         120U
#define BUZZER_PERIOD_MS     500U
static uint8_t buzzer_pulses;
static uint32_t buzzer_started_at;
static void Alarm_SetOutput(bool on)
{
    if (on)
    {
        PTA->PDDR |= BUZZER_MASK;
#if LCD_SDA_PIN != ALARM_LED_PIN
        PTC->PSOR = ALARM_LED_MASK;
#endif
        PTD->PDDR |= SHIELD_LED_MASK;
    }
    else
    {
        PTA->PDDR &= ~BUZZER_MASK;
#if LCD_SDA_PIN != ALARM_LED_PIN
        PTC->PCOR = ALARM_LED_MASK;
#endif
        PTD->PDDR &= ~SHIELD_LED_MASK;
    }
}

static void Buzzer_Play(uint8_t pulses, uint32_t now)
{
    buzzer_pulses = pulses;
    buzzer_started_at = now;
}

static void Buzzer_Stop(void)
{
    buzzer_pulses = 0U;
    Alarm_SetOutput(false);
}

static void Buzzer_Update(uint32_t now)
{
    if (buzzer_pulses == 0U)
    {
        return;
    }

    const uint32_t elapsed = now - buzzer_started_at;
    const uint32_t duration = (buzzer_pulses - 1U) * BUZZER_PERIOD_MS +
                              BUZZER_ON_MS;
    if (elapsed >= duration)
    {
        Buzzer_Stop();
    }
    else
    {
        Alarm_SetOutput((elapsed % BUZZER_PERIOD_MS) < BUZZER_ON_MS);
    }
}

// -----------------------------------------------------------------------------
// Countdown mode
// -----------------------------------------------------------------------------

#define COUNTDOWN_BEEP_SECONDS 3U
static const uint16_t digit_step[4] = {600U, 60U, 10U, 1U};
static void Increment_Digit(void)
{
    const uint32_t step = digit_step[selected_digit];
    const uint32_t limit = (selected_digit == 2U) ? 6U : 10U;
    const uint32_t digit = (seconds / step) % limit;
    if (digit + 1U == limit)
    {
        seconds -= digit * step;
    }
    else
    {
        seconds += step;
    }

    Display_Time();
}

static void Countdown_Edit(bool select, bool increment)
{
    if (select)
    {
        selected_digit = (uint8_t)((selected_digit + 1U) % 4U);
        blink_enabled = true;
        Display_Time();
    }

    if (increment)
    {
        Increment_Digit();
    }
}

static void Countdown_Advance(uint32_t elapsed, uint32_t now)
{
    seconds = (elapsed >= seconds) ? 0U : seconds - elapsed;
    if (seconds == 0U)
    {
        Buzzer_Play(BUZZER_FINISH_BEEPS, now);
        timer_state = IDLE;
    }
    else if (seconds <= COUNTDOWN_BEEP_SECONDS)
    {
        Buzzer_Play(1U, now);
    }
}

// -----------------------------------------------------------------------------
// Stopwatch mode
// -----------------------------------------------------------------------------

static void Stopwatch_Advance(uint32_t elapsed)
{
    seconds = (elapsed >= MAX_SECONDS - seconds) ?
              MAX_SECONDS : seconds + elapsed;
    if (seconds == MAX_SECONDS)
    {
        timer_state = IDLE;
    }
}

// -----------------------------------------------------------------------------
// Shared timer controls
// -----------------------------------------------------------------------------

static void Timer_Update(uint32_t now)
{
    if (timer_state != RUNNING)
    {
        return;
    }

    const uint32_t elapsed = (now - last_second_tick) / SECOND_MS;
    if (elapsed == 0U)
    {
        return;
    }

    last_second_tick += elapsed * SECOND_MS;
    if (mode == COUNTDOWN)
    {
        Countdown_Advance(elapsed, now);
    }
    else
    {
        Stopwatch_Advance(elapsed);
    }

    Display_Time();
}

static void Timer_StartPause(uint32_t now)
{
    if (timer_state == RUNNING)
    {
        paused_ms = now - last_second_tick;
        timer_state = PAUSED;
        Buzzer_Stop();
    }
    else if (timer_state == PAUSED)
    {
        last_second_tick = now - paused_ms;
        timer_state = RUNNING;
    }
    else if ((mode == COUNTDOWN && seconds != 0U) ||
             (mode == STOPWATCH && seconds < MAX_SECONDS))
    {
        last_second_tick = now;
        timer_state = RUNNING;
        blink_enabled = false;
        Display_Time();
        if (mode == COUNTDOWN && seconds <= COUNTDOWN_BEEP_SECONDS)
        {
            Buzzer_Play(1U, now);
        }
        else
        {
            Buzzer_Stop();
        }
    }
}

static void Toggle_Mode(void)
{
    Buzzer_Stop();
    timer_state = IDLE;
    selected_digit = 0U;
    blink_enabled = false;
    if (mode == COUNTDOWN)
    {
        saved_countdown = seconds;
        mode = STOPWATCH;
        seconds = 0U;
    }
    else
    {
        mode = COUNTDOWN;
        seconds = saved_countdown;
    }

    Display_Time();
}

static void Timer_Reset(void)
{
    Buzzer_Stop();
    timer_state = IDLE;
    seconds = 0U;
    saved_countdown = 0U;
    selected_digit = 0U;
    blink_enabled = false;
    Display_Time();
}

// -----------------------------------------------------------------------------
// Buttons and SW3 interrupt
// -----------------------------------------------------------------------------

#define DEBOUNCE_MS 25U
typedef struct
{
    bool raw_pressed;
    bool stable_pressed;
    uint32_t changed_at;
} Button;
static Button s1;
static Button s2;
static Button s3;
static Button sw2;
static Button sw3;
static volatile uint32_t sw3_edges;
static void SW3_InterruptInit(void)
{
    PORTA->PCR[SW3_PIN] |= PORT_PCR_IRQC(SW3_FALLING_IRQ);
    PORTA->ISFR = SW3_MASK;
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

void PORTA_IRQHandler(void)
{
    if ((PORTA->ISFR & SW3_MASK) != 0U)
    {
        PORTA->ISFR = SW3_MASK;
        sw3_edges++;
    }
}

static bool Button_Pressed(Button *button, bool pressed, uint32_t now)
{
    if (pressed != button->raw_pressed)
    {
        button->raw_pressed = pressed;
        button->changed_at = now;
    }

    if (pressed != button->stable_pressed &&
        (now - button->changed_at) >= DEBOUNCE_MS)
    {
        button->stable_pressed = pressed;
        return pressed;
    }

    return false;
}

static bool SW3_Pressed(uint32_t now)
{
    static uint32_t observed_edges;
    static bool debouncing;
    const uint32_t edges = sw3_edges;
    if (edges != observed_edges)
    {
        observed_edges = edges;
        debouncing = true;
    }

    if (!debouncing)
    {
        return false;
    }

    const bool pressed = (PTA->PDIR & SW3_MASK) == 0U;
    const bool reset = Button_Pressed(&sw3, pressed, now);
    if (!pressed && !sw3.stable_pressed)
    {
        debouncing = false;
    }

    return reset;
}

static void Buttons_Update(uint32_t now)
{
    const uint32_t levels_b = PTB->PDIR;
    const bool select = Button_Pressed(
        &s1, (levels_b & (1UL << S1_PIN)) == 0U, now);
    const bool increment = Button_Pressed(
        &s2, (levels_b & (1UL << S2_PIN)) == 0U, now);
    const bool start = Button_Pressed(
        &s3, (levels_b & (1UL << S3_PIN)) == 0U, now);
    const bool change_mode = Button_Pressed(
        &sw2, (PTC->PDIR & (1UL << SW2_PIN)) == 0U, now);
    const bool reset = SW3_Pressed(now);
    if (reset)
    {
        Timer_Reset();
        return;
    }

    if (change_mode)
    {
        Toggle_Mode();
        return;
    }

    if (mode == COUNTDOWN && timer_state == IDLE)
    {
        Countdown_Edit(select, increment);
    }

    if (start)
    {
        Timer_StartPause(now);
    }
}

// -----------------------------------------------------------------------------
// Startup diagnostics and main loop
// -----------------------------------------------------------------------------

static void LCD_BootDiagnostics(void)
{
    // Hold shield S1 while restarting to show the code for 5 seconds.
    if ((PTB->PDIR & (1UL << S1_PIN)) != 0U)
    {
        return;
    }

    const bool ready = LCD_Update(ms_ticks);
    const uint32_t lines = PTC->PDIR;
    uint32_t code = 0U;
    if (ready)
    {
        // Add the decimal I2C address to 1000.
        code = 1000U + lcd_address;
    }
    else
    {
        if ((lines & LCD_SDA_MASK) == 0U)
        {
            code |= 1U;
        }

        if ((lines & LCD_SCL_MASK) == 0U)
        {
            code |= 2U;
        }

        if (code == 0U)
        {
            code = (lcd_address == 0U) ? 4U : 5U;
        }
    }

    display_frame = ((code / 1000U) << 12U) |
                    (((code / 100U) % 10U) << 8U) |
                    (((code / 10U) % 10U) << 4U) |
                    (code % 10U);
    Delay_Ms(5000U);
    s1.raw_pressed = true;
    s1.stable_pressed = true;
    s1.changed_at = ms_ticks;
    Display_Time();
}

int main(void)
{
    GPIO_Init();
    SystemCoreClockUpdate();
    Clock_Init();
    SW3_InterruptInit();
    i2c_delay_cycles = (SystemCoreClock + 999999U) / 1000000U * I2C_DELAY_US;
    Delay_Ms(50U);
    LCD_BootDiagnostics();
    for (;;)
    {
        Clock_Update();
        const uint32_t now = ms_ticks;
        Timer_Update(now);
        Buttons_Update(now);
        Buzzer_Update(now);
        LCD_Update(now);
    }
}
