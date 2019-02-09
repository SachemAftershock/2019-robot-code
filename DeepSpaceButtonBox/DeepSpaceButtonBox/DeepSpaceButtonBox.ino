
// DeepSpaceButtonBox
// for Teensy++ 2.0 microcontroller to look like an XBox Controller (joystick in this library)
// https://www.pjrc.com/teensy/td_joystick.html
// Pushbuttons should be connected to digital pins 0 through 14.
// Wire each button between the digital pin and ground.
// You must select a Joystick type from the "Tools > USB Type" menu

// Define possible button states from digitalRead()
// Negative logic because pull-up resistor feature is used.
// Connect a switch lead to ground, and the other to the input pin.
const int ARCADE_BUTTON_PRESSED = 0;
const int ARCADE_BUTTON_RELEASED = 1;

const int JOYSTICK_BUTTON_PRESSED = 1;
const int JOYSTICK_BUTTON_RELEASED = 0;

// Buttons to command Elevator Floors w/ implicit base heights 
// and required angle of attacks relative to downfield 0 az.
// The numbers here represent the Teensy's DIO's. Silkscreen D0..7, 
// then just keep counting as if all were Dx through Ex, Cx etc per Arduino pin numbering.
const int ArcadeButton1_Floor = 0; 
const int ArcadeButton2_RocketLowerLeft = 1;
const int ArcadeButton3_RocketLowerMiddle = 2;
const int ArcadeButton4_RocketLowerRight = 3;
const int ArcadeButton5_RocketMediumLeft = 4;
const int ArcadeButton6_RocketMediumMiddle = 5;
const int ArcadeButton7_RocketMediumRight = 43; // DIO D6 seems to be stuck LOW, so keeps triggering. Must be because its the LED pin Pullup isn't working.
const int ArcadeButton8_RocketHighLeft = 7;
const int ArcadeButton9_RocketHighMiddle = 8;
const int ArcadeButton10_RocketHighRight = 9;
const int ArcadeButton11_CargoLeft = 10;
const int ArcadeButton12_CargoFront = 11;
const int ArcadeButton13_CargoRight = 12;
const int ArcadeButton14_HumanPlayerStation = 13;

// Button to command Grabber Orientation, and implicit offsets 
// to elevator base heights. 
const int ArcadeButton15_CargoHatchToggle = 14;

const int NumElevatorPositions = ArcadeButton14_HumanPlayerStation+1;

int Elevator_LUT[] = { 
  ArcadeButton1_Floor,
  ArcadeButton2_RocketLowerLeft,
  ArcadeButton3_RocketLowerMiddle,
  ArcadeButton4_RocketLowerRight,
  ArcadeButton5_RocketMediumLeft,
  ArcadeButton6_RocketMediumMiddle,
  ArcadeButton7_RocketMediumRight,
  ArcadeButton8_RocketHighLeft,
  ArcadeButton9_RocketHighMiddle,
  ArcadeButton10_RocketHighRight,
  ArcadeButton11_CargoLeft,
  ArcadeButton12_CargoFront,
  ArcadeButton13_CargoRight,
  ArcadeButton14_HumanPlayerStation,
  ArcadeButton15_CargoHatchToggle
};


// Keep track of the last commanded values. 
// NOTE: Must make sure these initialize to match what the RoboRIO code will initialize too.
// TODO: at this time, there is no provision for the Teensy and RoboRIO to re-synchronize these 
// last commanded value state, which would be preferred feedback from the RoboRIO.  Find a way 
// for RoboRIO to communicate back to this HID device to drive these variables and the LEDs.
int LastLevelButtonPressed = ArcadeButton3_RocketLowerMiddle;
enum enumCargoHatchToggle { eHatch = 0, eCargo = 1 } lastCargoHatchToggle = eHatch;

// Each elevator level button has one LED positioned underneath it.
//NOTE: Floor LED is not implemented, as it is a momentary action
const int LED_01_Floor = 15;
const int LED_02_RocketLowerLeft = 16;
const int LED_03_RocketLowerMiddle =17;
const int LED_04_RocketLowerRight = 18;
const int LED_05_RocketMediumLeft = 19;
const int LED_06_RocketMediumMiddle = 20;
const int LED_07_RocketMediumRight = 21;
const int LED_08_RocketHighLeft = 22;
const int LED_09_RocketHighMiddle = 23;
const int LED_10_RocketHighRight = 24;
const int LED_11_CargoLeft = 25;
const int LED_12_CargoFront = 38;
const int LED_13_CargoRight = 39;
const int LED_14_HumanPlayerStation = 40;

// The Cargo/Hatch Toggle button has a LED on each side, 
// to indicate which is the current setting.
const int LED_15_Cargo = 41;
const int LED_16_Hatch = 42;

int LED_LUT[] = { 
  LED_01_Floor,
  LED_02_RocketLowerLeft,
  LED_03_RocketLowerMiddle,
  LED_04_RocketLowerRight,
  LED_05_RocketMediumLeft,
  LED_06_RocketMediumMiddle,
  LED_07_RocketMediumRight,
  LED_08_RocketHighLeft,
  LED_09_RocketHighMiddle,
  LED_10_RocketHighRight,
  LED_11_CargoLeft,
  LED_12_CargoFront,
  LED_13_CargoRight,
  LED_14_HumanPlayerStation,
  LED_15_Cargo,
  LED_16_Hatch
};

int previousToggleIteration = ARCADE_BUTTON_RELEASED;

void setup() {
  
  Serial.begin(9600);
  
  // Configure DIOs for Rocket & Cargo Bay & human station buttons (one button per).
  // Each button drives distinct elevator level base target
  pinMode(ArcadeButton1_Floor, INPUT_PULLUP);  
  pinMode(ArcadeButton2_RocketLowerLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton3_RocketLowerMiddle, INPUT_PULLUP); 
  pinMode(ArcadeButton4_RocketLowerRight, INPUT_PULLUP); 
  pinMode(ArcadeButton5_RocketMediumLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton6_RocketMediumMiddle, INPUT_PULLUP); 
  pinMode(ArcadeButton7_RocketMediumRight, INPUT_PULLUP); 
  pinMode(ArcadeButton8_RocketHighLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton9_RocketHighMiddle, INPUT_PULLUP); 
  pinMode(ArcadeButton10_RocketHighRight, INPUT_PULLUP); 
  pinMode(ArcadeButton11_CargoLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton12_CargoFront, INPUT_PULLUP); 
  pinMode(ArcadeButton13_CargoRight, INPUT_PULLUP); 
  pinMode(ArcadeButton14_HumanPlayerStation, INPUT_PULLUP); 

  // Toggle for Cargo/Hatch offsets for each elevator level target.  The offset will be 
  // added to the base level identified by the Base Level Target. 
  pinMode(ArcadeButton15_CargoHatchToggle, INPUT_PULLUP); 

  // Configure DIOs for each LED on the box.
  pinMode(LED_01_Floor, OUTPUT);  
  pinMode(LED_02_RocketLowerLeft, OUTPUT); 
  pinMode(LED_03_RocketLowerMiddle, OUTPUT); 
  pinMode(LED_04_RocketLowerRight, OUTPUT); 
  pinMode(LED_05_RocketMediumLeft, OUTPUT); 
  pinMode(LED_06_RocketMediumMiddle, OUTPUT); 
  pinMode(LED_07_RocketMediumRight, OUTPUT); 
  pinMode(LED_08_RocketHighLeft, OUTPUT); 
  pinMode(LED_09_RocketHighMiddle, OUTPUT); 
  pinMode(LED_10_RocketHighRight, OUTPUT); 
  pinMode(LED_11_CargoLeft, OUTPUT); 
  pinMode(LED_12_CargoFront, OUTPUT); 
  pinMode(LED_13_CargoRight, OUTPUT); 
  pinMode(LED_14_HumanPlayerStation, OUTPUT); 

  pinMode(LED_15_Cargo, OUTPUT); 
  pinMode(LED_16_Hatch, OUTPUT); 

  setAllLEDS();
  delay(1000);
  clearAllLEDS();
  digitalWrite(LED_12_CargoFront, HIGH);
  digitalWrite(LED_16_Hatch, HIGH);
  
  Serial.println("Setup Complete");
}

void loop() {
  
  // Written so that USB messages only sent if there is a button pressed and it is
  // changed from the last button pressed.
  
  // Scan the elevator level buttons for a pressed button.
  for (int i=0; i<NumElevatorPositions; i++){
    // find first elevator level button that is pressed (that ain't same as 
    // previous), go with that one.
    if (digitalRead(Elevator_LUT[i]) == ARCADE_BUTTON_PRESSED){
        if (Elevator_LUT[i] != Elevator_LUT[LastLevelButtonPressed]){
          clearAllXboxControllerElevatorButtons();
          SetXboxControllerButton(i, JOYSTICK_BUTTON_PRESSED); 
          digitalWrite(LED_LUT[LastLevelButtonPressed], LOW);
          LastLevelButtonPressed = i;
          digitalWrite(LED_LUT[LastLevelButtonPressed], HIGH);
          Serial.print("Elevator Button Pressed: Arcade DIO#");
          Serial.print(Elevator_LUT[i]);
          Serial.print(", Joystick Button #");
          Serial.print(LastLevelButtonPressed+1);
          Serial.println(".");
          break;
      }
      else break;
    }
  }

  // Inspect the Hatch/Cargo button whether a toggle is to be commanded.
  if (digitalRead(ArcadeButton15_CargoHatchToggle) == ARCADE_BUTTON_PRESSED) {
    // Make sure to ignore until button is released, don't keep re-commanding if leaning on button.
    if (previousToggleIteration == ARCADE_BUTTON_RELEASED) {  
      previousToggleIteration = ARCADE_BUTTON_PRESSED;
      switch (lastCargoHatchToggle) {
        case eHatch:
          lastCargoHatchToggle = eCargo;
          digitalWrite(LED_15_Cargo, HIGH);
          digitalWrite(LED_16_Hatch, LOW);
          break;
        case eCargo:
          lastCargoHatchToggle = eHatch;
          digitalWrite(LED_15_Cargo, LOW);
          digitalWrite(LED_16_Hatch, HIGH);
          break;
      }
      SetXboxControllerButton(ArcadeButton15_CargoHatchToggle, lastCargoHatchToggle);
      Serial.print("Cargo/Hatch Button Pressed, commanded: ");
      Serial.println(lastCargoHatchToggle+1);
    }
  }
  else
  {
    previousToggleIteration = ARCADE_BUTTON_RELEASED;
  }

  // a brief delay, so this runs 20 times per second
  delay(50);
}

void clearAllXboxControllerElevatorButtons() {
  for (int arcadeButtonIndex=0; arcadeButtonIndex<NumElevatorPositions; arcadeButtonIndex++){
    SetXboxControllerButton(arcadeButtonIndex, JOYSTICK_BUTTON_RELEASED);
  }
}

void setAllLEDS() {
  for (int led : LED_LUT){
    digitalWrite(led, HIGH);
  }
}
void clearAllLEDS() {
  for(int led : LED_LUT) {
    digitalWrite(led, LOW);
  }
}

void SetXboxControllerButton(int logicalOrderedArcadeButton, int pressedState) {
  Joystick.button(logicalOrderedArcadeButton+1, pressedState);
}
