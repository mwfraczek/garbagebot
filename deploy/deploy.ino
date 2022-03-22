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

// Accelerometer Variables
double quat_x, quat_y, quat_z; // Quaternion corrected acceleration values
double g_x = 0, g_y = 0, g_z = -1000; // Vector used to store the gravity values and rotate the subsequent acceleration onto.
double abs_x, abs_y, abs_z; // Absolute orientation vector
double norm_x = 0, norm_y = 0, norm_z = 0; // Used to hold acceleration normalization values
float accelema = 0.01; // exponential moving average for acceleration.
double nrmrel_x = 0, nrmrel_y = 0, nrmrel_z = 0; // Used to hold normalization reliability values
double calib_x = 0, calib_y = 0, calib_z = 0; // Used to hold calibrated accelerometer values
float compema = 0.15; // ema for the compass
float compass = 0; // current compass
float avgcomp = 0; // averagecompass value

// Pathing Variables
int pilotsignal = 2000; // variable for pilotsignal transmitter
bool greenlight = false; // is bot in autonomous mode?
bool panicking = false; // are we emergency stopped?
double speed_y = 0; // y speed
int speedmax = 4; // speed limiter
double pos_y = 0; // y position variables
int bound_y = 40; // length of the area to be cleared
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
  success &= (Accel1.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
  success &= (Accel1.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);

  // For correct sensor fusion of the quaternion and the acceleration in this example, we need them to be running at the same rate.
  // Winding the data rate up too far results in instability in the DMP.
  int fusionrate = 4;
  success &= (Accel1.setDMPODRrate(DMP_ODR_Reg_Quat6, fusionrate) == ICM_20948_Stat_Ok); // Set to 225Hz
  success &= (Accel1.setDMPODRrate(DMP_ODR_Reg_Accel, fusionrate) == ICM_20948_Stat_Ok); // Set to 225Hz
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

  findGravity();

  // Reset pathing vars
  pos_y = 0;
  speed_y = 0;
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
    else if (PWM_L < 1430 && PWM_L !=0) //LEFT joystick is forward, map PWM values, send signal
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
    calcAccel();
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
void calcAccel()
{
  int moreData = readIMUDMP(); // Read data from accelerometer

  if (moreData >= 0) //function returns -1 if no data was read.
  {

    //Quat 6 sensor range set to 16g - if this is exceeded, then the quarternion will lose it's lock on which way is down. So slowly update gravity with the latest readings?
    bool updategravity = true;
    if (updategravity and (sqrt(quat_x * quat_x + quat_y * quat_y + quat_z * quat_z) < 1080)) // We can only update the gravity vector when the sensor is relatively stationary.
    {
      g_x = g_x * (1 - accelema) + quat_x * accelema;
      g_y = g_y * (1 - accelema) + quat_y * accelema;
      g_z = g_z * (1 - accelema) + quat_z * accelema;
    }

    // Calculates the average value of the accelerometers (i.e. the internal bias)
    norm_x = norm_x * (1 - accelema) + abs_x * accelema;
    norm_y = norm_y * (1 - accelema) + abs_y * accelema;
    norm_z = norm_z * (1 - accelema) + abs_z * accelema;

    // This subtracts the accelerometer bias from the absolute values
    calib_x = abs_x - norm_x;
    calib_y = abs_y - norm_y;
    calib_z = abs_z - norm_z;

    // The reliability of the Norm values (i.e. how close is the average of the calibrated accel values to 0)
    nrmrel_x = nrmrel_x * (1 - accelema) + calib_x * accelema;
    nrmrel_y = nrmrel_y * (1 - accelema) + calib_y * accelema;
    nrmrel_z = nrmrel_z * (1 - accelema) + calib_z * accelema;

    // Average compass value
    avgcomp = avgcomp * (1 - compema) + compass * compema;

    /*  // Prints final values to serial
        SERIAL_PORT.print(F(" Accel: X:"));
        SERIAL_PORT.print(calib_x);
        SERIAL_PORT.print(F(" Y:"));
        SERIAL_PORT.print(calib_y);
        SERIAL_PORT.print(F(" Z:"));
        SERIAL_PORT.print(calib_z);

        SERIAL_PORT.print(F(" Norms: NX:"));
        SERIAL_PORT.print(nrmrel_x);
        SERIAL_PORT.print(F(" NY:"));
        SERIAL_PORT.print(nrmrel_y);
        SERIAL_PORT.print(F(" NZ:"));
        SERIAL_PORT.println(nrmrel_z);
    */
  }

  if (moreData <= 0) {
    //Wait if there is no more data available
    delay(10);
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

    // Update Acceleration
    if (((data.header & DMP_header_bitmap_Quat6) > 0) || ((data.header & DMP_header_bitmap_Accel) > 0)) // Check for simulataneous quaternion and accel data
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt( 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      float acc_y = (float)data.Raw_Accel.Data.Y;
      float acc_z = (float)data.Raw_Accel.Data.Z;


      /*
        Uncomment the following for raw data output from the Quaternion and the Accelerometer
        SERIAL_PORT.print(F("Q0:"));
        SERIAL_PORT.print(q0, 3);
        SERIAL_PORT.print(F(" Q1:"));
        SERIAL_PORT.print(q1, 3);
        SERIAL_PORT.print(F(" Q2:"));
        SERIAL_PORT.print(q2, 3);
        SERIAL_PORT.print(F(" Q3:"));
        SERIAL_PORT.print(q3, 3);

        SERIAL_PORT.print(F(" Accel: X:"));
        SERIAL_PORT.print(acc_x);
        SERIAL_PORT.print(F(" Y:"));
        SERIAL_PORT.print(acc_y);
        SERIAL_PORT.print(F(" Z:"));
        SERIAL_PORT.println(acc_z);
      */


      // Quaternion is initialised as q1=q2=q3=0. Perform Quaternion rotation of raw acceleration values into the initial sensor rotation frame:

      quat_x = acc_x * (1 - 2 * q2 * q2 - 2 * q3 * q3) + acc_y * 2 * (q1 * q2 - q0 * q3) + acc_z * 2 * (q1 * q3 + q0 * q2);
      quat_y = acc_x * 2 * (q1 * q2 + q0 * q3) + acc_y * (1 - 2 * q1 * q1 - 2 * q3 * q3) + acc_z * 2 * (q2 * q3 - q0 * q1);
      quat_z = acc_x * 2 * (q1 * q3 - q0 * q2) + acc_y * 2 * (q2 * q3 + q0 * q1) + acc_z * (1 - 2 * q1 * q1 - 2 * q2 * q2);

      // Re-orient the sensor so that gravity is down. See: https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
      // Basically, we want the quaternion rotation to rotate the whole system so that the gravity vector (x,y,z) maps onto the unit vector (0,0,-1)
      // The cross-product of the gravity and the "down" vector is (-y,x,0) and the dot-product is (-z).
      // The rotation quaternion is therefore given by (q0,q1,q2,q3) = (sqrt(g_x*g_x+g_y*g_y+g_z*g_z)-g_z , -g_y , g_x , 0); although this needs normalising before application.

      // Re-use the previous variables from Quat rotation - this might not be best practice
      q0 = sqrt(g_x * g_x + g_y * g_y + g_z * g_z) - g_z;
      q1 = -g_y;
      q2 = g_x;
      q3 = 0;
      double quatMag = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

      q0 /= quatMag;
      q1 /= quatMag;
      q2 /= quatMag;
      q3 /= quatMag;

      abs_x = quat_x * (1 - 2 * q2 * q2 - 2 * q3 * q3) + quat_y * 2 * (q1 * q2 - q0 * q3) + quat_z * 2 * (q1 * q3 + q0 * q2);
      abs_y = quat_x * 2 * (q1 * q2 + q0 * q3) + quat_y * (1 - 2 * q1 * q1 - 2 * q3 * q3) + quat_z * 2 * (q2 * q3 - q0 * q1);
      abs_z = quat_x * 2 * (q1 * q3 - q0 * q2) + quat_y * 2 * (q2 * q3 + q0 * q1) + quat_z * (1 - 2 * q1 * q1 - 2 * q2 * q2);


      bool subtractG = true;  // Do you want to subtract the gravity reading (true), or just orient the sensor (false)
      if (subtractG)
      {
        abs_z += sqrt(g_x * g_x + g_y * g_y + g_z * g_z);
      }
    }

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

void findGravity()
{
  //Serial.println("Calibrating gravity vector - keep the sensor still or this might give inaccurate results");
  //Do exponential moving average over a certain time period and wait until vector settles at around 1g. Takes around 8 seconds.
  double totalAcc = sqrt(g_x * g_x + g_y * g_y + g_z * g_z);
  // Average the first xxx readings.DMP seems to take a few seconds to settle down anyhow.
  int counter = 4000;
  int i = 0;
  double grav_x = 0, grav_y = 0, grav_z = 0;

  while (i < counter)
  {
    int moreData = readIMUDMP();
    if (moreData >= 0) //function returns -1 if no data was read.
    {
      grav_x += quat_x / counter;
      grav_y += quat_y / counter;
      grav_z += quat_z / counter;
      i += 1;
    }

    if (moreData <= 0) {
      delay(1);
    }
  }
  g_x = grav_x;
  g_y = grav_y;
  g_z = grav_z;
}

void calcPos()
{
  // placeholder for speed and position calculation
  delay(1);

}

// calibrate compass and speed if nessecary
void calibrate()
{
  // Set path compass
  for (int index = 0; index <= 100; index++)
    {
      int moreData = readIMUDMP();
      if (moreData >= 0) //function returns -1 if no data was read.
      {
        avgcomp = avgcomp * (1 - compema) + compass * compema;
      }
      if (moreData <= 0)
      {
        delay(1);
      }
    }
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
    // while within y bound, drive forward
    if (pos_y < bound_y)
    {
      autodriveForward(); // drive forward
      turning = false;
    }
    else
    {
      if (turning == false)
      {
        allStop(); // stop driving
        depositScoop(); // collect from scoop
        turning = true;
        driveline = false;
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
  if (speed_y < speedmax)
  {
    PWM_L += pwrstep;
    PWM_R += pwrstep;

    PWM_L = constrain(PWM_L, PWMMIN, PWMMAX);
    PWM_R = constrain(PWM_R, PWMMIN, PWMMAX);
  }

  // If speed over max, decrease power, constrain to PWM
  if (speed_y > speedmax)
  {
    PWM_L -= pwrstep;
    PWM_R -= pwrstep;

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
  // Set PWMs to 0
  PWM_L = 0;
  PWM_R = 0;

  // Write values to motors
  analogWrite(IN1, -1);
  analogWrite(AN1, PWM_L);
  analogWrite(IN2, -1);
  analogWrite(AN2, PWM_R);
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
    deployScoop();
    turning = false;
    turnComplete = false;
    LR = 1;
    pos_y = 0;
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
    deployScoop();
    turning = false;
    turnComplete = false;
    LR = 0;
    pos_y = 0;
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
void deployScoop()
{
  // placeholder for scoop deploy subroutine
  delay(1);
}

void adjustScoop()
{
  // placeholder for scoop adjust subroutine
  delay(1);
}

void depositScoop()
{
  // placeholder for deposit subroutine
  delay(1);
}
