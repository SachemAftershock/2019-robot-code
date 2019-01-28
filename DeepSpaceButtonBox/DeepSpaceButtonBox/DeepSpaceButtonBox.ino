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
const int ArcadeButton0_Floor = 0; 
const int ArcadeButton1_RocketLowerLeft = 1;
const int ArcadeButton2_RocketLowerMiddle = 2;
const int ArcadeButton3_RocketLowerRight = 3;
const int ArcadeButton4_RocketMediumLeft = 4;
const int ArcadeButton5_RocketMediumMiddle = 5;
const int ArcadeButton6_RocketMediumRight = 41; // My DIO D6 seems to be stuck LOW, so keeps triggering.
const int ArcadeButton7_RocketHighLeft = 7;
const int ArcadeButton8_RocketHighMiddle = 8;
const int ArcadeButton9_RocketHighRight = 9;
const int ArcadeButton10_CargoLeft = 10;
const int ArcadeButton11_CargoFront = 11;
const int ArcadeButton12_CargoRight = 12;
const int ArcadeButton13_HumanPlayerStation = 13;

// Button to command Grabber Orientation, and implicit offsets 
// to elevator base heights. 
const int ArcadeButton14_CargoHatchToggle = 14;

const int NumElevatorPositions = ArcadeButton13_HumanPlayerStation+1;

int Elevator_LUT[] = { 
  ArcadeButton0_Floor,
  ArcadeButton1_RocketLowerLeft,
  ArcadeButton2_RocketLowerMiddle,
  ArcadeButton3_RocketLowerRight,
  ArcadeButton4_RocketMediumLeft,
  ArcadeButton5_RocketMediumMiddle,
  ArcadeButton6_RocketMediumRight,
  ArcadeButton7_RocketHighLeft,
  ArcadeButton8_RocketHighMiddle,
  ArcadeButton9_RocketHighRight,
  ArcadeButton10_CargoLeft,
  ArcadeButton11_CargoFront,
  ArcadeButton12_CargoRight,
  ArcadeButton13_HumanPlayerStation,
  ArcadeButton14_CargoHatchToggle
};


// Keep track of the last commanded values. 
// NOTE: Must make sure these initialize to match what the RoboRIO code will initialize too.
// TODO: at this time, there is no provision for the Teensy and RoboRIO to re-synchronize these 
// last commanded value state, which would be preferred feedback from the RoboRIO.  Find a way 
// for RoboRIO to communicate back to this HID device to drive these variables and the LEDs.
int LastLevelButtonPressed = ArcadeButton2_RocketLowerMiddle;
enum enumCargoHatchToggle { eHatch = 0, eCargo = 1 } lastCargoHatchToggle = eHatch;

// Each elevator level button has one LED positioned underneath it.
const int LED_00_Floor = 15;
const int LED_01_RocketLowerLeft = 16;
const int LED_02_RocketLowerMiddle =17;
const int LED_03_RocketLowerRight = 18;
const int LED_04_RocketMediumLeft = 19;
const int LED_05_RocketMediumMiddle = 20;
const int LED_06_RocketMediumRight = 21;
const int LED_07_RocketHighLeft = 22;
const int LED_08_RocketHighMiddle = 23;
const int LED_09_RocketHighRight = 24;
const int LED_10_CargoLeft = 25;
const int LED_11_CargoFront = 26;
const int LED_12_CargoRight = 27;
const int LED_13_HumanPlayerStation = 38;

// The Cargo/Hatch Toggle button has a LED on each side, 
// to indicate which is the current setting.
const int LED_14_Cargo = 39;
const int LED_15_Hatch = 40;

int LED_LUT[] = { 
  LED_00_Floor,
  LED_01_RocketLowerLeft,
  LED_02_RocketLowerMiddle,
  LED_03_RocketLowerRight,
  LED_04_RocketMediumLeft,
  LED_05_RocketMediumMiddle,
  LED_06_RocketMediumRight,
  LED_07_RocketHighLeft,
  LED_08_RocketHighMiddle,
  LED_09_RocketHighRight,
  LED_10_CargoLeft,
  LED_11_CargoFront,
  LED_12_CargoRight,
  LED_13_HumanPlayerStation,
  LED_14_Cargo,
  LED_15_Hatch
};

int previousToggleIteration = ARCADE_BUTTON_RELEASED;

void setup() {
  
  Serial.begin(9600);
  
  // Configure DIOs for Rocket & Cargo Bay & human station buttons (one button per).
  // Each button drives distinct elevator level base target
  pinMode(ArcadeButton0_Floor, INPUT_PULLUP);  
  pinMode(ArcadeButton1_RocketLowerLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton2_RocketLowerMiddle, INPUT_PULLUP); 
  pinMode(ArcadeButton3_RocketLowerRight, INPUT_PULLUP); 
  pinMode(ArcadeButton4_RocketMediumLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton5_RocketMediumMiddle, INPUT_PULLUP); 
  pinMode(ArcadeButton6_RocketMediumRight, INPUT_PULLUP); 
  pinMode(ArcadeButton7_RocketHighLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton8_RocketHighMiddle, INPUT_PULLUP); 
  pinMode(ArcadeButton9_RocketHighRight, INPUT_PULLUP); 
  pinMode(ArcadeButton10_CargoLeft, INPUT_PULLUP); 
  pinMode(ArcadeButton11_CargoFront, INPUT_PULLUP); 
  pinMode(ArcadeButton12_CargoRight, INPUT_PULLUP); 
  pinMode(ArcadeButton13_HumanPlayerStation, INPUT_PULLUP); 

  // Toggle for Cargo/Hatch offsets for each elevator level target.  The offset will be 
  // added to the base level identified by the Base Level Target. 
  pinMode(ArcadeButton14_CargoHatchToggle, INPUT_PULLUP); 

  // Configure DIOs for each LED on the box.
  pinMode(LED_00_Floor, OUTPUT);  
  pinMode(LED_01_RocketLowerLeft, OUTPUT); 
  pinMode(LED_02_RocketLowerMiddle, OUTPUT); 
  pinMode(LED_03_RocketLowerRight, OUTPUT); 
  pinMode(LED_04_RocketMediumLeft, OUTPUT); 
  pinMode(LED_05_RocketMediumMiddle, OUTPUT); 
  pinMode(LED_06_RocketMediumRight, OUTPUT); 
  pinMode(LED_07_RocketHighLeft, OUTPUT); 
  pinMode(LED_08_RocketHighMiddle, OUTPUT); 
  pinMode(LED_09_RocketHighRight, OUTPUT); 
  pinMode(LED_10_CargoLeft, OUTPUT); 
  pinMode(LED_11_CargoFront, OUTPUT); 
  pinMode(LED_12_CargoRight, OUTPUT); 
  pinMode(LED_13_HumanPlayerStation, OUTPUT); 

  pinMode(LED_14_Cargo, OUTPUT); 
  pinMode(LED_15_Hatch, OUTPUT); 
  
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
  if (digitalRead(ArcadeButton14_CargoHatchToggle) == ARCADE_BUTTON_PRESSED) {
    // Make sure to ignore until button is released, don't keep re-commanding if leaning on button.
    if (previousToggleIteration == ARCADE_BUTTON_RELEASED) {  
      previousToggleIteration = ARCADE_BUTTON_PRESSED;
      switch (lastCargoHatchToggle) {
        case eHatch:
          lastCargoHatchToggle = eCargo;
          digitalWrite(LED_14_Cargo, HIGH);
          digitalWrite(LED_15_Hatch, LOW);
          break;
        case eCargo:
          lastCargoHatchToggle = eHatch;
          digitalWrite(LED_14_Cargo, LOW);
          digitalWrite(LED_15_Hatch, HIGH);
          break;
      }
      SetXboxControllerButton(ArcadeButton14_CargoHatchToggle, lastCargoHatchToggle);
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

void SetXboxControllerButton(int logicalOrderedArcadeButton, int pressedState) {
  Joystick.button(logicalOrderedArcadeButton+1, pressedState);
}
