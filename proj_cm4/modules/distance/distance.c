#include "distance.h"
#include "HCSR04.h"
#include "cybsp.h"
#include <stddef.h>

// distance [mm] = time [us] * speed [m/s] / 2000, the sound goes there and back
#define ROUND_TRIP_DIVIDER (2000u)

// Cuts longer pulses, so that the multiplication cannot overflow
#define ECHO_US_LIMIT (1000000u)

// A measurement without a valid distance. Bigger than any real one, so the median counts it as "very far"
#define NO_ECHO (0xFFFFu)

#if (DISTANCE_FILTER_SIZE < 1u) || (DISTANCE_FILTER_SIZE > 15u) || ((DISTANCE_FILTER_SIZE % 2u) == 0u)
#error "DISTANCE_FILTER_SIZE must be odd, from 1 to 15"
#endif

/* Written by the filter, read in the main loop. In the automatic mode the filter runs in the interrupt. */
static uint16_t window[DISTANCE_FILTER_SIZE];
static uint8_t window_size = 0;
static uint8_t window_next = 0;

static Distance_Mode mode = DISTANCE_MODE_AUTO;
static uint32_t last_driver_number = 0;
static volatile uint32_t measurement_count = 0;
static volatile uint16_t raw_mm = NO_ECHO;
static volatile uint16_t filtered_mm = NO_ECHO;

static void processMeasurement(bool has_echo, uint32_t echo_us);

bool Distance_Init(Distance_Mode init_mode)
{
  if ((init_mode != DISTANCE_MODE_AUTO && init_mode != DISTANCE_MODE_MANUAL) || !HCSR04_Init())
  {
    return false;
  }

  mode = init_mode;
  HCSR04_setMeasurementCallback(mode == DISTANCE_MODE_AUTO ? processMeasurement : NULL);
  return true;
}

void Distance_Start(void)
{
  HCSR04_Stop();

  uint32_t interrupt_state = Cy_SysLib_EnterCriticalSection();
  window_size = 0;
  window_next = 0;
  last_driver_number = 0;
  measurement_count = 0;
  raw_mm = NO_ECHO;
  filtered_mm = NO_ECHO;
  Cy_SysLib_ExitCriticalSection(interrupt_state);

  HCSR04_Start();
}

void Distance_Stop(void)
{
  HCSR04_Stop();
}

uint32_t Distance_EchoMicrosecondsToMm(uint32_t echo_us)
{
  if (echo_us > ECHO_US_LIMIT)
  {
    echo_us = ECHO_US_LIMIT;
  }
  return (echo_us * DISTANCE_SOUND_SPEED_MPS + ROUND_TRIP_DIVIDER / 2u) / ROUND_TRIP_DIVIDER;
}

static void addToWindow(uint16_t measurement_mm)
{
  window[window_next] = measurement_mm;
  window_next = (uint8_t)((window_next + 1u) % DISTANCE_FILTER_SIZE);
  if (window_size < DISTANCE_FILTER_SIZE)
  {
    ++window_size;
  }
}

/* While the window is filling up, the median of what is there is used */
static uint16_t medianOfWindow(void)
{
  uint16_t sorted[DISTANCE_FILTER_SIZE];

  for (uint8_t i = 0; i < window_size; ++i)
  {
    uint16_t value = window[i];
    uint8_t j = i;
    while (j > 0 && sorted[j - 1] > value)
    {
      sorted[j] = sorted[j - 1];
      --j;
    }
    sorted[j] = value;
  }
  return sorted[(window_size - 1u) / 2u];
}

/* Automatic mode: called from the interrupt of the HCSR04 driver. Manual mode: called by Distance_Update(). */
static void processMeasurement(bool has_echo, uint32_t echo_us)
{
  uint16_t measurement_mm = NO_ECHO;
  if (has_echo)
  {
    uint32_t mm = Distance_EchoMicrosecondsToMm(echo_us);
    if (mm >= DISTANCE_MIN_MM && mm <= DISTANCE_MAX_MM)
    {
      measurement_mm = (uint16_t)mm;
    }
  }

  raw_mm = measurement_mm;
  addToWindow(measurement_mm);
  filtered_mm = medianOfWindow();
  ++measurement_count;
}

bool Distance_Update(void)
{
  if (mode != DISTANCE_MODE_MANUAL)
  {
    return false;
  }

  uint32_t echo_us;
  uint32_t driver_number;
  bool has_echo = HCSR04_getEcho(&echo_us, &driver_number);

  if (driver_number == last_driver_number)
  {
    return false;
  }
  last_driver_number = driver_number;

  processMeasurement(has_echo, has_echo ? echo_us : 0u);
  return true;
}

static bool valueToResult(uint16_t value, uint16_t *distance_mm)
{
  if (value == NO_ECHO)
  {
    return false;
  }
  if (distance_mm != NULL)
  {
    *distance_mm = value;
  }
  return true;
}

bool Distance_GetMm(uint16_t *distance_mm)
{
  return valueToResult(filtered_mm, distance_mm);
}

bool Distance_GetRawMm(uint16_t *distance_mm)
{
  return valueToResult(raw_mm, distance_mm);
}

uint32_t Distance_GetCount(void)
{
  return measurement_count;
}
