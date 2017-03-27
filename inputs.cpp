#include "inputs.h"

Adafruit_MCP23017 ioExpander;

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

  // The data and address pins are on the IO expander, so it the write switch
  ioExpander.begin();

  // Because the Adafruit library hides the writeRegister function (it's private)
  // we can't configure the io expander in two transmissions
  //ioExpander.writeRegister(MCP23017_GPPUA, 0x00);
  //ioExpander.writeRegister(MCP23017_GPPUB, 0x00);

  // Instead we have to do 16 transitions. Urgh!
  for (int i = 0; i<= 15; i++){
    ioExpander.pullUp(i,1);
  }
}

bool isWriteEnabled(){
  return ioExpander.readGPIO(0) & 0x10;
}

int getDataInput(){
  return ioExpander.readGPIO(1);  
}

int getAddrInput(){
  int rawInput = ioExpander.readGPIO(0);
  // Hmph! I got the wiring backwards.
  return 
  ((rawInput & 128)>>7)|
  ((rawInput &  64)>>5)|
  ((rawInput &  32)>>3);
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
  Serial.print("Pwr:");
  Serial.print(powerIsOn());
  Serial.print(" | ");

  Serial.print("Brt:");
  Serial.print(getBrightnessChange());
  Serial.print(" | ");

  Serial.print("View:");
  Serial.print(getViewMode());
  Serial.print(" | ");

  Serial.print("Clk:");
  Serial.print(getClockMode());
  Serial.print(" | ");

  Serial.print("Usr:");
  Serial.print(isInUserMode());
  Serial.print(" | ");

  Serial.print("Rst:");
  Serial.print(isInReset());
  Serial.print(" | ");

  Serial.print("Stp:");
  Serial.print(isStepHeld());
  Serial.print(" | ");

  Serial.print("Run:");
  Serial.print(isRunOn());
  Serial.print(" | ");

  Serial.print("Wrt:");
  Serial.print(isWriteEnabled());
  Serial.print(" | ");

  Serial.print("Addr:");
  Serial.print(getAddrInput());
  Serial.print(" | ");

  Serial.print("Data:");
  Serial.print(getDataInput());
  Serial.println("");
}

