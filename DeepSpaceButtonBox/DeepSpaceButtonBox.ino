// DeepSpaceButtonBox
// for Teensy++ 2.0 microcontroller to look like an XBox Controller (joystick in this library)
// https://www.pjrc.com/teensy/td_joystick.html
// Pushbuttons should be connected to digital pins 0 through 14.
// Wire each button between the digital pin and ground.
// You must select a Joystick type from the "Tools > USB Type" menu

// Define possible button states from digitalRead()
const int BUTTON_PRESSED = 1;
const int BUTTON_RELEASED = 0;

const int Button0_Floor = 0;
const int Button1_RocketLowerLeft = 1;
const int Button2_RocketLowerMiddle = 2;
const int Button3_RocketLowerRight = 3;
const int Button4_RocketMediumLeft = 4;
const int Button5_RocketMediumMiddle = 5;
const int Button6_RocketMediumRight = 6;
const int Button7_RocketHighLeft = 7;
const int Button8_RocketHighMiddle = 8;
const int Button9_RocketHighRight = 9;
const int Button10_CargoLeft = 10;
const int Button11_CargoFront = 11;
const int Button12_CargoRight = 12;
const int Button13_HumanPlayerStation = 13;

const int Button14_CargoHatchToggle = 14;

const int NumElevatorPositions = Button13_HumanPlayerStation+1;

const int InvalidLevel = -1;
int LastLevelButtonPressed = InvalidLevel;

enum enumCargoHatchToggle { eHatch = 0, eCargo = 1, eInvalid = 2 } lastCargoHatchToggle = eInvalid;

void setup() {
  
  // Configure DIOs for Rocket & Cargo Bay & human station buttons (one button per).
  // Each button drives distinct elevator level base target
  pinMode(Button0_Floor, INPUT_PULLUP);  
  pinMode(Button1_RocketLowerLeft, INPUT_PULLUP); 
  pinMode(Button2_RocketLowerMiddle, INPUT_PULLUP); 
  pinMode(Button3_RocketLowerRight, INPUT_PULLUP); 
  pinMode(Button4_RocketMediumLeft, INPUT_PULLUP); 
  pinMode(Button5_RocketMediumMiddle, INPUT_PULLUP); 
  pinMode(Button6_RocketMediumRight, INPUT_PULLUP); 
  pinMode(Button7_RocketHighLeft, INPUT_PULLUP); 
  pinMode(Button8_RocketHighMiddle, INPUT_PULLUP); 
  pinMode(Button9_RocketHighRight, INPUT_PULLUP); 
  pinMode(Button10_CargoLeft, INPUT_PULLUP); 
  pinMode(Button11_CargoFront, INPUT_PULLUP); 
  pinMode(Button12_CargoRight, INPUT_PULLUP); 
  pinMode(Button13_HumanPlayerStation, INPUT_PULLUP); 

  // Toggle for Cargo/Hatch offsets for each elevator level target.  The offset will be 
  // added to the base level identified by the Base Level Target. 
  pinMode(Button14_CargoHatchToggle, INPUT_PULLUP); 

}

void loop() {
  
  // Written so that USB messages only sent if there is a button pressed and it is
  // changed from the last button pressed.
  
  //int readValue = -1;
  
  // Scan the elevator level buttons for a pressed button.
  for (int i=0; i<NumElevatorPositions; i++){
    // find first elevator level button that is pressed (that ain't same as 
    // previous), go with that one.
    if ((digitalRead(i) == BUTTON_PRESSED) && (i != LastLevelButtonPressed)) {
      Joystick.button(i, BUTTON_PRESSED);
      LastLevelButtonPressed = i;
      break;
    }
  }

  // Inspect the Hatch/Cargo button whether a toggle is to be commanded.
  if (digitalRead(Button14_CargoHatchToggle) == BUTTON_PRESSED) {
    // Boolean/logical operators in C are required to yield either 0 or 1. 
    // From section 6.5.3.3/5 of the ISO C99 standard.  The result of the 
    // logical negation operator ! is 0 if the value of its operand compares 
    // unequal to 0, 1 if the value of its operand compares equal to 0.
    // https://stackoverflow.com/questions/3661751/what-is-0-in-c
    lastCargoHatchToggle = !lastCargoHatchToggle;  
    Joystick.button(Button14_CargoHatchToggle, lastCargoHatchToggle);
  }

  // a brief delay, so this runs 20 times per second
  delay(50);
}
