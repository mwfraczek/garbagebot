// Libraries
#include <ICM_20948.h> // Accellerometer Library (SparkFun 9DoF IMU Breakout)

// Definitions
ICM_20948_I2C Accel1;  // Creates Accelerometer Object in I2C
#define AD0_VAL 0 // Value of the last bit of the I2C address
#define WIRE_PORT Wire
#define SERIAL_PORT Serial

// Variables
// Pins
int AN2 = 5;       //PIN -- PWM output (right)
int AN1 = 6;       //PIN -- PWM output (left)
int IN2 = 7;       //PIN -- Motor direction output (right)
int IN1 = 8;       //PIN -- Motor direction output (left)
int RC_R = 10;     //PIN -- RC input   (right), Channel 2
int RC_L = 3;      //PIN -- RC input   (left), Channel 3
int pilotbutton = 52; // Pin to give the greenlight for autopath, Channel 9
int emergencystop = 53; // Emergency shutoff pin
int ultraLtrig = 22, ultraLecho = 23; // left ultrasound pins
int ultraRtrig = 24, ultraRecho = 25; // right ultrasound pins
int ultraStrig = 26, ultraSecho = 27; // scoop ultrasound pins

// Drive Motor PWMs
int PWM_R;         //SIGNAL RIGHT
int PWM_L;         //SIGNAL LEFT
int PWMMIN = 55, PWMMAX = 100; // min and max PWM values
int pwrstep = 1; // how large a change of pwr per step

// Compass Variables
float compema = 0.15; // ema for the compass
float compass = 0; // current compass
float avgcomp = 0; // averagecompass value

// Pathing Variables
int pilotsignal = 2000; // variable for pilotsignal transmitter
bool greenlight = false; // is bot in autonomous mode?
bool panicking = false; // are we emergency stopped?
unsigned long drivetime = 0; // Current drivetime
unsigned long maxdrivetime = 10000; // Maximum drivetime (milliseconds)
int turns = 0, maxturns = 10; // how many columns to clear
int LR = 1; // will bot turn left or right next? 0 = left, 1 = right
float pathcompass = 0; // path compass value
int compassconstraint = 2; // min and max deviation of compass for autodriveForward
bool driveline = false; // are we driving forward?
bool turning = false; // are we turning?
bool turnComplete = false; // is the turn complete?
bool calibrated = false; // are we calibrated?

// Ultrasound Variables
float ultraDistanceL = 0; // left ultrasound, raw distance
float ultraAvgL = 0; // left ultrasound, averaged for noise
float ultraDistanceR = 0; // right ultrasound, raw distance
float ultraAvgR = 0; // right ultrasound, averaged for noise
int ultraObstacle = 100; // check for obstacles within this distance

// Scoop Variables
bool scoopdeployed = false;
float scoopheight = 0;


void setup() {
  // Setup Pins
  pinMode(RC_R, INPUT);
  pinMode(RC_L, INPUT);
  pinMode(pilotbutton, INPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(ultraLtrig, OUTPUT);// left ultrasound pins
  pinMode(ultraLecho, INPUT);
  pinMode(ultraRtrig, OUTPUT);// right ultrasound pins
  pinMode(ultraRecho, INPUT);
  pinMode(ultraStrig, OUTPUT);// scoop ultrasound pins
  pinMode(ultraSecho, INPUT);
  attachInterrupt(digitalPinToInterrupt(emergencystop), panic, HIGH); // emergency stop interrupt
  // Setup Serial and I2C
  SERIAL_PORT.begin(115200);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  // Disable autonomous mode
  greenlight = false;
  panicking = false;

  // Initialize the IMU
  bool initialized = false;
  while (!initialized)
  {
    Accel1.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(Accel1.statusString());
    if (Accel1.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  bool success = true; // Use success to show if the DMP configuration was successful

  success &= (Accel1.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable DMP Sensors
  success &= (Accel1.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

  // Set the data rate of the geomagnetic compass
  int fusionrate = 4;
  success &= (Accel1.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, fusionrate) == ICM_20948_Stat_Ok); // Set to 225Hz

  // Enable the FIFO
  success &= (Accel1.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (Accel1.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (Accel1.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (Accel1.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }

  // Reset pathing vars
  drivetime = 0;
  turns = 0;
  LR = 1;
  pathcompass = 0;
  driveline = false;
  turning = false;
  turnComplete = false;
  calibrated = false;
}

void loop() {
  // RC input to toggle greenlight via pilotbutton
  pilotsignal = pulseIn(pilotbutton, HIGH);
  if (pilotsignal > 1900 || pilotsignal == 0 || panicking == true)
  {
    greenlight = false;
  }
  if (pilotsignal < 1900 && pilotsignal != 0 && panicking == false)
  {
    greenlight = true;
  }

  // When not in autonomous mode
  if (greenlight = false)
  {
    PWM_L = pulseIn(RC_L, HIGH); //take RC input signals
    PWM_R = pulseIn(RC_R, HIGH);

    // LEFT joystick control //
    if ((PWM_L >= 1430 && PWM_L <= 1540) || PWM_L == 0) // LEFT joystick is centered (neither fwd/reverse)
    {
      analogWrite(AN1, 0);
    }
    else if (PWM_L > 1540) //LEFT joystick is in reverse, map PWM values, send signal
    {
      analogWrite(IN1, 1);
      PWM_L = map(PWM_L, 1539, 1915, 55, 255);
      analogWrite(AN1, PWM_L);
    }
    else if (PWM_L < 1430 && PWM_L != 0) //LEFT joystick is forward, map PWM values, send signal
    {
      analogWrite(IN1, -1);
      PWM_L = map(PWM_L, 1431, 1080, 55, 255);
      analogWrite(AN1, PWM_L);
    }

    // RIGHT joystick control //
    if ((PWM_R >= 1430 && PWM_R <= 1540) || PWM_R == 0) // RIGHT joystick is centered (neither fwd/reverse)
    {
      analogWrite(AN2, 0);
    }
    else if (PWM_R > 1540) //RIGHT joystick is in reverse, map PWM values, send signal
    {
      analogWrite(IN2, 1);
      PWM_R = map(PWM_R, 1539, 1915, 55, 255);
      analogWrite(AN2, PWM_R);
    }
    else if (PWM_R < 1430 && PWM_R != 0) //RIGHT joystick is forward,  map PWM values, send signal
    {
      analogWrite(IN2, -1);
      PWM_R = map(PWM_R, 1431, 1080, 55, 255);
      analogWrite(AN2, PWM_R);
    }
  }
  // When in autonomous mode
  if (greenlight = true)
  {
    if (calibrated = false)
    {
      calibrate();
    }
    calcIMU();
    calcPos();
    ultraSound();
    adjustScoop();
    autoPath();
  }
}

// Emergency Stop ISR. Returns to manual control
void panic()
{
  greenlight = false;
  panicking = true;
}

// IMU Funtions
void calcIMU()
{
  int moreData = readIMUDMP(); // Read data from accelerometer

  if (moreData >= 0) //function returns -1 if no data was read.
  {
    // Average compass value
    avgcomp = avgcomp * (1 - compema) + compass * compema;
  }
}

int readIMUDMP() {
  // Get all IMU Data
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  Accel1.readDMPdataFromFIFO(&data);

  if ((Accel1.status == ICM_20948_Stat_Ok) || (Accel1.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    // Update Compass
    if ((data.header & DMP_header_bitmap_Compass_Calibr) > 0) // Check for Compass
    {
      compass = (float)data.Compass_Calibr.Data.Y / 10000; // Extract the compass data
    }
  }
  if (Accel1.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    return 0; // No more data
  }
  else
  {
    return 1; // More data
  }
}

void calcPos()
{
  // Calculate drivetime
  if (driveline == false && turning == false)
  {
    drivetime = millis();
    driveline = true;
  }

  // If over drivetime, set driving to false
  if (((millis() - drivetime) > maxdrivetime))
  {
    driveline = false;
  }
}

// calibrate compass and set path direction
void calibrate()
{
  // Calibrate compass
  for (int index = 0; index <= 100; index++)
  {
    int moreData = readIMUDMP();
    if (moreData >= 0) //function returns -1 if no data was read.
    {
      avgcomp = avgcomp * (1 - compema) + compass * compema;
    }
  }

  // Set path direction
  pathcompass = avgcomp;
  calibrated = true;
}

// Pathing Functions
void autoPath()
{
  // Path code
  // If not in autonomous mode, return
  if (greenlight == false)
  {
    return;
  }

  // Keep going until max turns reached
  if (turns < maxturns)
  {
    // while within time bound, drive forward
    if (driveline == true)
    {
      autodriveForward(); // drive forward
      turning = false;
    }
    else
    {
      if (turning == false)
      {
        allStop(); // stop driving
        turning = true;
      }
    }

    if (turnComplete == false && turning == true)
    {
      // Turn left or right
      switch (LR)
      {
        case 0:
          autoturnLeft(); // turn left
          break;

        case 1:
          autoturnRight();  // turn right
          break;
      }
    }
  }
  else
  {
    allStop();
  }
}


// Forward drive path
void autodriveForward()
{
  // If speed under max, increase power, constrain to PWM
  if (PWM_L < PWMMAX || PWM_R < PWMMAX)
  {
    PWM_L += pwrstep;
    PWM_R += pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // If drifting right, correct left
  if (abs(pathcompass - avgcomp) > compassconstraint)
  {
    PWM_L -= 2 * pwrstep;
    PWM_R += 2 * pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // If drifting left, correct right
  if (abs(avgcomp - pathcompass) > compassconstraint)
  {
    PWM_L += 2 * pwrstep;
    PWM_R -= 2 * pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // Write values to motors
  analogWrite(IN1, -1);
  analogWrite(AN1, PWM_L);
  analogWrite(IN2, -1);
  analogWrite(AN2, PWM_R);
}

void allStop()
{
  // Slowly set PWMs to 0
  while (PWM_L > 0 || PWM_R > 0)
  {
    PWM_L -= pwrstep;
    PWM_R -= pwrstep;

    PWM_L = constrain(PWM_L, 0, PWMMAX);
    PWM_R = constrain(PWM_R, 0, PWMMAX);

    // Write values to motors
    analogWrite(IN1, -1);
    analogWrite(AN1, PWM_L);
    analogWrite(IN2, -1);
    analogWrite(AN2, PWM_R);
  }
}

void autoturnLeft()
{
  if ((avgcomp / pathcompass) > -0.98 || (avgcomp / pathcompass) < -1.02)
  {

    PWM_R += pwrstep;
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);

    analogWrite(IN2, -1);
    analogWrite(AN2, PWM_R);
  }
  else
  {
    turnComplete = true;
  }
  if (turnComplete == true)
  {
    allStop();
    driveline = false;
    turning = false;
    turnComplete = false;
    LR = 1;
    turns += 1;
    pathcompass = -pathcompass;
  }
}

void autoturnRight()
{
  if ((avgcomp / pathcompass) > -0.98 || (avgcomp / pathcompass) < -1.02)
  {

    PWM_L += pwrstep;
    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);

    analogWrite(IN1, -1);
    analogWrite(AN1, PWM_L);
  }
  else
  {
    turnComplete = true;
  }
  if (turnComplete == true)
  {
    allStop();
    driveline = false;
    turning = false;
    turnComplete = false;
    LR = 0;
    turns += 1;
    pathcompass = -pathcompass;
  }
}

// Sensor functions
void ultraSound()
{
  // If scoop deployed, driving, and object within 3 feet, disable greenlight
  // placeholder for forward ultrasound sensor readings
  delay(1);
}

// Scoop Functions
void adjustScoop()
{
  // placeholder for scoop adjust subroutine
  delay(1);
}

void deployScoop()
{
  // placeholder for scoop deploy subroutine
  delay(1);
}

void depositScoop()
{
  // placeholder for deposit subroutine
  delay(1);
}
