#include "HCSR04.h"
#include "cybsp.h"
#include "cy_sysint.h"

#define ECHO_PORT (ULTRA_ECHO_PORT)
#define ECHO_PIN  (ULTRA_ECHO_PIN)
#define ECHO_IRQ  (ULTRA_ECHO_IRQ)

#define TIMER_HW     (PWM_Trig_HW)
#define TIMER_NUM    (PWM_Trig_NUM)
#define TIMER_MASK   (PWM_Trig_MASK)
#define TIMER_CONFIG (PWM_Trig_config)
#define TIMER_IRQ    (PWM_Trig_IRQ)

/* The clock channels of the TCPWM0 counters follow one another */
#define TIMER_CLOCK ((en_clk_dst_t) (PCLK_TCPWM0_CLOCKS0 + TIMER_NUM))

/* The same priority, so that the two interrupts cannot interrupt each other while the callback runs */
#define ECHO_IRQ_PRIORITY  (2u)
#define TIMER_IRQ_PRIORITY (2u)

#define MICROSECONDS_PER_SECOND (1000000u)

#define TRIG_PULSE_MIN_US     (10u)
#define PERIOD_MIN_US         (40000u)
#define MEASUREMENT_PERIOD_US (50000u)

static uint32_t ticks_per_us = 0;
static bool initialized = false;
static HCSR04_MeasurementCallback measurement_callback = NULL;

static volatile uint32_t echo_rise_ticks = 0;
static volatile bool echo_is_high = false;
static volatile bool echo_seen_in_period = false;
static volatile bool echo_is_valid = false;
static volatile uint32_t last_echo_us = 0;
static volatile uint32_t measurement_count = 0;

/* Every finished measurement is stored for HCSR04_getEcho() and given to the callback, if there is one */
static void finishMeasurement(bool has_echo, uint32_t echo_us) {
    echo_is_valid = has_echo;
    last_echo_us = echo_us;
    ++measurement_count;

    if (measurement_callback != NULL) {
        measurement_callback(has_echo, echo_us);
    }
}

static void echoIsr(void) {
    if (0UL == Cy_GPIO_GetInterruptStatusMasked(ECHO_PORT, ECHO_PIN)) {
        return;
    }

    /* Read the counter first to keep the delay between the edge and the timestamp short */
    uint32_t now_ticks = Cy_TCPWM_PWM_GetCounter(TIMER_HW, TIMER_NUM);
    bool level_is_high = (0UL != Cy_GPIO_Read(ECHO_PORT, ECHO_PIN));
    Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);

    if (level_is_high) {
        echo_rise_ticks = now_ticks;
        echo_is_high = true;
        return;
    }

    if (!echo_is_high || now_ticks < echo_rise_ticks) {
        /* Falling edge without a rising edge, or the counter restarted between the edges */
        echo_is_high = false;
        return;
    }
    echo_is_high = false;
    echo_seen_in_period = true;

    finishMeasurement(true, (now_ticks - echo_rise_ticks) / ticks_per_us);
}

/* Runs at the start of every period, which is when the next Trig pulse starts */
static void timerIsr(void) {
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);

    if (!echo_seen_in_period) {
        /* An empty measurement is a finished measurement too */
        finishMeasurement(false, 0);
    }
    /* Do not let an incomplete Echo pulse span a timer wrap and look like a short pulse. */
    echo_is_high = false;
    echo_seen_in_period = false;
}

/* The tick length comes from the clock divider chosen in the Device Configurator */
static bool readTickRate(void) {
    uint32_t assigned = Cy_SysClk_PeriphGetAssignedDivider(TIMER_CLOCK);
    cy_en_divider_types_t divider_type =
            (cy_en_divider_types_t) ((assigned & PERI_CLOCK_CTL_TYPE_SEL_Msk) >> PERI_CLOCK_CTL_TYPE_SEL_Pos);
    uint32_t divider_num = (assigned & PERI_CLOCK_CTL_DIV_SEL_Msk) >> PERI_CLOCK_CTL_DIV_SEL_Pos;

    /* Prescaler values 0..7 mean divide by 1..128 */
    uint32_t tick_hz = Cy_SysClk_PeriphGetFrequency(divider_type, divider_num) >> TIMER_CONFIG.clockPrescaler;

    ticks_per_us = tick_hz / MICROSECONDS_PER_SECOND;
    return ticks_per_us != 0 && 0 == tick_hz % MICROSECONDS_PER_SECOND;
}

bool HCSR04_Init(void) {
    /* A too short Trig pulse or a period shorter than the longest echo would give wrong results */
    if (!readTickRate() || TIMER_CONFIG.compare0 < TRIG_PULSE_MIN_US * ticks_per_us ||
        TIMER_CONFIG.period0 < PERIOD_MIN_US * ticks_per_us) {
        return false;
    }

    if (CY_TCPWM_SUCCESS != Cy_TCPWM_PWM_Init(TIMER_HW, TIMER_NUM, &TIMER_CONFIG)) {
        return false;
    }
    Cy_TCPWM_PWM_SetPeriod0(TIMER_HW, TIMER_NUM, MEASUREMENT_PERIOD_US * ticks_per_us - 1u);
    Cy_TCPWM_Enable_Multiple(TIMER_HW, TIMER_MASK);

    /* The pins are configured by cybsp_init(), the Echo interrupt stays off until HCSR04_Start() */
    Cy_GPIO_SetInterruptMask(ECHO_PORT, ECHO_PIN, 0UL);
    Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);

    const cy_stc_sysint_t echo_irq_config = {
            .intrSrc = ECHO_IRQ,
            .intrPriority = ECHO_IRQ_PRIORITY,
    };
    const cy_stc_sysint_t timer_irq_config = {
            .intrSrc = TIMER_IRQ,
            .intrPriority = TIMER_IRQ_PRIORITY,
    };
    if (CY_SYSINT_SUCCESS != Cy_SysInt_Init(&echo_irq_config, echoIsr) ||
        CY_SYSINT_SUCCESS != Cy_SysInt_Init(&timer_irq_config, timerIsr)) {
        return false;
    }
    NVIC_EnableIRQ(ECHO_IRQ);
    NVIC_EnableIRQ(TIMER_IRQ);

    initialized = true;
    return true;
}

void HCSR04_Start(void) {
    if (!initialized) {
        return;
    }

    uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_SetInterruptMask(ECHO_PORT, ECHO_PIN, 0UL);
    Cy_TCPWM_SetInterruptMask(TIMER_HW, TIMER_NUM, 0UL);
    Cy_TCPWM_TriggerStopOrKill(TIMER_HW, TIMER_MASK);
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);
    Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);
    NVIC_ClearPendingIRQ(ECHO_IRQ);
    NVIC_ClearPendingIRQ(TIMER_IRQ);

    echo_is_high = false;
    echo_seen_in_period = false;
    echo_is_valid = false;
    last_echo_us = 0;
    measurement_count = 0;

    Cy_TCPWM_TriggerReloadOrIndex(TIMER_HW, TIMER_MASK);
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);
    Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);
    Cy_GPIO_SetInterruptMask(ECHO_PORT, ECHO_PIN, CY_GPIO_INTR_EN_MASK);
    Cy_TCPWM_SetInterruptMask(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);
    Cy_TCPWM_TriggerStart(TIMER_HW, TIMER_MASK);
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}

void HCSR04_Stop(void) {
    if (!initialized) {
        return;
    }

    uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_SetInterruptMask(ECHO_PORT, ECHO_PIN, 0UL);
    Cy_TCPWM_SetInterruptMask(TIMER_HW, TIMER_NUM, 0UL);
    Cy_TCPWM_TriggerStopOrKill(TIMER_HW, TIMER_MASK);
    Cy_TCPWM_ClearInterrupt(TIMER_HW, TIMER_NUM, CY_TCPWM_INT_ON_TC);
    Cy_GPIO_ClearInterrupt(ECHO_PORT, ECHO_PIN);
    NVIC_ClearPendingIRQ(ECHO_IRQ);
    NVIC_ClearPendingIRQ(TIMER_IRQ);
    echo_is_high = false;
    echo_seen_in_period = false;
    echo_is_valid = false;
    last_echo_us = 0;
    Cy_SysLib_ExitCriticalSection(interrupt_state);
}

void HCSR04_setMeasurementCallback(HCSR04_MeasurementCallback callback) {
    measurement_callback = callback;
}

bool HCSR04_getEcho(uint32_t* echo_us, uint32_t* measurement_number) {
    /* The interrupts change several variables, so read them together */
    uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
    bool valid = echo_is_valid;
    uint32_t value = last_echo_us;
    uint32_t number = measurement_count;
    Cy_SysLib_ExitCriticalSection(interrupt_state);

    if (valid && echo_us != NULL) {
        *echo_us = value;
    }
    if (measurement_number != NULL) {
        *measurement_number = number;
    }
    return valid;
}
