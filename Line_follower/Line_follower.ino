
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   12    // emitter is controlled by digital pin 2

#define M1 200
#define M2 200

#define KP 0.1
#define KD 5

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {2, 4, 6, 7, 8, 9, 10, 11},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
   
unsigned int sensorValues[NUM_SENSORS];

int VelA = 3;

// MOTOR B_RIGHT


int VelB = 5;
void setup() {
  // put your setup code here, to run once:
  delay(500);
  pinMode(13, OUTPUT);
  pinMode(VelA,OUTPUT);
  pinMode(VelB,OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}

int lastError = 0;
 
void loop()
{
  
  // get calibrated sensor values returned in the sensors array, along with the line
  // position, which will range from 0 to 2000, with 1000 corresponding to the line over
  // the middle sensor
  int position = qtr.readLine(sensorValues);
 
  // compute our "error" from the line position.  We will make it so that the error is zero
  // when the middle sensor is over the line, because this is our goal.  Error will range
  // from -1000 to +1000.  If we have sensor 0 on the left and sensor 2 on the right,  
  // a reading of -1000 means that we see the line on the left and a reading of +1000 
  // means we see the line on the right.
  int error = position - 4000;
 
  // set the motor speed based on proportional and derivative PID terms
  // KP is the a floating-point proportional constant (maybe start with a value around 0.1)
  // KD is the floating-point derivative constant (maybe start with a value around 5)
  // note that when doing PID, it's very important you get your signs right, or else the
  // control loop will be unstable
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
 
  // M1 and M2 are base motor speeds.  That is to say, they are the speeds the motors
  // should spin at if you are perfectly on the line with no error.  If your motors are
  // well matched, M1 and M2 will be equal.  When you start testing your PID loop, it
  // might help to start with small values for M1 and M2.  You can then increase the speed
  // as you fine-tune your PID constants KP and KD.
  int m1Speed = M1 + motorSpeed;
  int m2Speed = M2 - motorSpeed;
 
  // it might help to keep the speeds positive (this is optional)
  // note that you might want to add a similiar line to keep the speeds from exceeding
  // any maximum allowed value
  if (m1Speed < 0)
    m1Speed = 0;
  else if (m1Speed>255)
    m1Speed = 255;
 
  if (m2Speed < 0)
    m2Speed = 0;
  else if (m2Speed > 255)
    m2Speed = 255;
    
 
  // set motor speeds using the two motor speed variables above
  analogWrite(VelA,m1Speed);
  analogWrite(VelB,m2Speed);
  
}
