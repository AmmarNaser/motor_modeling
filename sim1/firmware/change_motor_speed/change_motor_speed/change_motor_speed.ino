/******************************************
   COURSE  : MTE504 MECHATRONICS II
   PROJECT : SPEED CHANGER
   DATE    : 04/08/2021
 *****************************************/

// Serial Port Used
#define ser Serial

// Motor PWM Pin
byte pinMotor = 5;

// Direvction Pin
byte pinDir1 = 7;
byte pinDir2 = 8;

void setup() {
  // Setup serial baudrate
  ser.begin(9600);

  // Set pins as output
  pinMode(pinMotor, OUTPUT);
  pinMode(pinDir1, OUTPUT);
   pinMode(pinDir2, OUTPUT);

  // Default to 0
  digitalWrite(pinMotor, LOW);
  digitalWrite(pinDir1, LOW);
   digitalWrite(pinDir2, LOW);
}

void loop() {
}

void serialEvent() {
  if (ser.available()) {
    char cmd = ser.read();
    switch (cmd) {
      case 's':
        // Set Speed to 100%
        analogWrite(pinMotor, 1.0 * 255.0);
        break;

      case 'c':
        // Clockwise
        digitalWrite(pinDir1, HIGH);
         digitalWrite(pinDir2, LOW);
        break;

      case 'a':
        // Anti Clockwise
        digitalWrite(pinDir2, HIGH);
         digitalWrite(pinDir1, LOW);
        break;

      case 'p':
        // Set Speed to 0%
        analogWrite(pinMotor, 0.0);
          digitalWrite(pinDir1, LOW);
   digitalWrite(pinDir2, LOW);
        
        break;

      default:
        break;
    }
  }
}