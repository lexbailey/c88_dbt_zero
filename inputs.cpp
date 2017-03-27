#include "inputs.h"

void initInputs(){
  pinMode(SWITCH_POWER, INPUT);
  pinMode(SWITCH_BRIGHT_INC, INPUT);
  pinMode(SWITCH_BRIGHT_DEC, INPUT);
  pinMode(SWITCH_VIEW_UP, INPUT);
  pinMode(SWITCH_VIEW_DOWN, INPUT);
  pinMode(SWITCH_CLOCK_UP, INPUT);
  pinMode(SWITCH_CLOCK_DOWN, INPUT);
  pinMode(SWITCH_USER, INPUT);
  pinMode(SWITCH_RESET, INPUT);
  pinMode(SWITCH_STEP, INPUT);
  pinMode(SWITCH_RUN, INPUT);
}

bool powerIsOn(){
  return digitalRead(SWITCH_POWER) == HIGH;
}

int getBrightnessChange(){
  if (digitalRead(SWITCH_BRIGHT_INC)  == HIGH) {return +1;}
  if (digitalRead(SWITCH_BRIGHT_DEC) == HIGH) {return -1;}  
  return 0;
}

int getViewMode(){
  if (digitalRead(SWITCH_VIEW_UP)   == HIGH) {return VIEW_PC;}
  if (digitalRead(SWITCH_VIEW_DOWN) == HIGH) {return VIEW_REG;}  
  return VIEW_MEM;
}

int getClockMode(){
  if (digitalRead(SWITCH_CLOCK_UP)   == HIGH) {return CLOCK_INPUT_FULL;}
  if (digitalRead(SWITCH_CLOCK_DOWN) == HIGH) {return CLOCK_INPUT_SLOW;}  
  return CLOCK_INPUT_FAST;
}

bool isInUserMode(){
  return digitalRead(SWITCH_USER) == HIGH;
}

bool isInReset(){
  return digitalRead(SWITCH_RESET) == HIGH;
}

bool isStepHeld(){
  return digitalRead(SWITCH_STEP) == HIGH;
}

bool isRunOn(){
  return digitalRead(SWITCH_RUN) == HIGH;
}

void debugInputs(){
  Serial.print("Power: ");
  Serial.print(powerIsOn());
  Serial.print(" | ");

  Serial.print("Bright: ");
  Serial.print(getBrightnessChange());
  Serial.print(" | ");

  Serial.print("View: ");
  Serial.print(getViewMode());
  Serial.print(" | ");

  Serial.print("Clock: ");
  Serial.print(getClockMode());
  Serial.print(" | ");

  Serial.print("User: ");
  Serial.print(isInUserMode());
  Serial.print(" | ");

  Serial.print("Reset: ");
  Serial.print(isInReset());
  Serial.print(" | ");

  Serial.print("Step: ");
  Serial.print(isStepHeld());
  Serial.print(" | ");

  Serial.print("Run: ");
  Serial.print(isRunOn());
  Serial.println("");
}

