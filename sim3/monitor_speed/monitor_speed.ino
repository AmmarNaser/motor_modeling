/******************************************
   COURSE  : MTE504 MECHATRONICS II
   PROJECT : MONITOR SPEED
   DATE    : 10/08/2021
 *****************************************/

/***** LIBRARIES *****/
#include <Encoder.h>
#include <TimerOne.h>
/********************/

/***** VARIABLES *****/

// Serial Port Used
#define PORT Serial

// PWM value
int valuePWM = 0;

// Motor PWM Pin
byte pinMotor = 5;

// Direvction Pin
byte pinDir1 = 7;
byte pinDir2 = 8;

// ENCODER
byte pinCHA = 2;
byte pinCHB = 3;

// Encoder number  of pulses per revolution
float PulsesPerRevolution = 380;

// Calculated speed in RPM
volatile float speedRPM = 0;

// Current count of incoming encoder pulses
volatile float PulseCount = 0.0;

// TIMER
float timerInterval = 0.1;

/********************/

// Timer Interrupt Service Routine
void tmrISR()
{
  // Disable interrupt
  noInterrupts();
  speedRPM = ((PulseCount / timerInterval) / PulsesPerRevolution) * 60.0;
  PORT.print(valuePWM);
  PORT.print(",");
  PORT.println(speedRPM);
  PulseCount = 0;
  // Enable Interrupt
  interrupts();
}

// External Interrupt Service Routine
void externalISR()
{
  // Increment pulse count
  PulseCount++;
}

void setup()
{
  // Setup serial baudrate
  PORT.begin(115200);

  // Set pins as output
  pinMode(pinMotor, OUTPUT);
  pinMode(pinDir1, OUTPUT);
  pinMode(pinDir2, OUTPUT);

  // Default to 0
  digitalWrite(pinMotor, LOW);
  digitalWrite(pinDir1, LOW);
  digitalWrite(pinDir2, LOW);
  // Encoder PIN
  pinMode(pinCHA, INPUT); // Set encoder pin as input to receive pulse train
}

void loop()
{
}

void serialEvent()
{
  // Read command characters until \n is received
  auto IncomingCommand = PORT.readStringUntil('\n');
  const auto Command = IncomingCommand.substring(0, IncomingCommand.indexOf(','));
  auto StringData = IncomingCommand.substring(IncomingCommand.indexOf(',') + 1, IncomingCommand.length());
  PORT.flush();
  // Parsing command
  if (Command == "GO")
  {
    // Start streaming data
    Timer1.initialize(timerInterval * 1000000);
    //Attach ISR
    Timer1.attachInterrupt(tmrISR);

    //Enable external interrupt (Rising Edge)
    attachInterrupt(digitalPinToInterrupt(pinCHA), externalISR, RISING);
  }
  if (Command == "END")
  {
    // Stop Motor
    analogWrite(pinMotor, 0.0);
    //Detach ISR
    Timer1.detachInterrupt();
    detachInterrupt(digitalPinToInterrupt(pinCHA));
  }

  // New command for PWM
  if (Command == "PWM")
  {
    // Send PWM value
    valuePWM = StringData.toInt();
    analogWrite(pinMotor, valuePWM);
  }

  // New command for direction
  if (Command == "DIR1")
  {
  digitalWrite(pinDir1, HIGH);
  digitalWrite(pinDir2, LOW);    
    }
     if (Command == "DIR2")
  {
  digitalWrite(pinDir2, HIGH);
  digitalWrite(pinDir1, LOW);    
    }
  }

