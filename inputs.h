#include <Arduino.h>

#define SWITCH_POWER (2)
#define SWITCH_BRIGHT_INC (4)
#define SWITCH_BRIGHT_DEC (3)
#define SWITCH_VIEW_UP (5)
#define SWITCH_VIEW_DOWN (6)
#define SWITCH_CLOCK_UP (8)
#define SWITCH_CLOCK_DOWN (7)
#define SWITCH_USER (9)
#define SWITCH_RESET (A0)
#define SWITCH_STEP (A1)
#define SWITCH_RUN (A2)

#define VIEW_PC  (0)
#define VIEW_MEM (1)
#define VIEW_REG (2)

#define CLOCK_INPUT_SLOW (0)
#define CLOCK_INPUT_FAST (1)
#define CLOCK_INPUT_FULL (2)

void initInputs();

bool powerIsOn();

int getBrightnessChange();

int getViewMode();

int getClockMode();

bool isInUserMode();

bool isInReset();

bool isStepHeld();

bool isRunOn();

void debugInputs();
