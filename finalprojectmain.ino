// This #include statement was automatically added by the Particle IDE.
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include <helper_3dmath.h>
#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <I2Cdev.h>
#include <math.h>
#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined (PARTICLE)
    #include "Wire.h"
#endif
//______________________________________________________________________________________________________________________________________________________
// global variables for the MPU using the MPU6050_DMP6 code
#define OUTPUT_READABLE_YAWPITCHROLL

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
/*for this program, we will be using ypr, specifically yaw to calculate rotation. in future implimatations,
we can use all 3 axes to determine the user's orientation more accurately*/
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//__________________________________________________________________________________________________________________________________________________________


//particle pins
const int echoPin = D4;
const int trigPin = D5;
const int servopin = D3;
const int buttonpin = D7;
const int buzzerpin = D6;
/* MPU pins are automatically set by Wire.h
            MPU SCL = D1
            MPU SDA = D0
            MPU INT = D2
*/

//_________________________________________________________________________________________________________________________________________________________

const int servoSpeed = 5;
const int servoRotationMax = 180; //rotation in degrees

//these values should be editied based on the prefrence of the user
//these values are in inches
const double arm_length = 4; //this is the distance from the center of torso to the tip of finger (MUST BE SET BEFORE USE OF DEVICE)
double activation_distance = 20; //this is the distance the ultrasonic sensor must detect in order to start its rotation | same as arm_length for simplicity of coding change later
double buffer_distance = 2; //this is the allowed distance between user and sensor when the servo motor maxes out its rotation

uint32_t duration_us;
int readingsLen = 10;
float readings[10];
int readingsIndex = 0;

float startOrientation = NULL;

int systemTimer = 0;

Servo servoMotor;
MPU6050 mpu;


//============================================================================================================================================================
 
void setup() {
    //MPU Setup
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		#if defined (PARTICLE)
			Wire.setSpeed(CLOCK_SPEED_400KHZ);
			Wire.begin();
		#else
			Wire.begin();
			TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		#endif
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    #if !defined (PARTICLE)
        while (!Serial); // wait for Leonardo enumeration, others continue immediately
    #endif
    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
	#if defined (PARTICLE)
        attachInterrupt(D2, dmpDataReady, RISING);
	#else
        attachInterrupt(0, dmpDataReady, RISING);
	#endif
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    //______________________________________________________________________
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWriteFast(trigPin, LOW);
    
    servoMotor.attach(servopin);  
    servoMotor.write(0);
    
    pinMode(buttonpin, INPUT);
    
    pinMode(buzzerpin,OUTPUT);//set buzzerPin as OUTPUT
    digitalWrite(buzzerpin, HIGH);
    
    runMPU();
    startOrientation = ypr[0] * 180/M_PI;
    /*ring the buzzer 10 times. the purpose of this is to indeicate 
    to the user that they need to let the MPU set itself to get accurate readings*/
    //ringBuzzer(1000, 0);
}

//=========================================================================================================================

void loop() {
    checkButton();//checks for a button press to set the distance of the user from objects, and resets the current orientation of the user
    pingUltrasonic();//this calculates the distance of the user from an object
    calculateServoRotation(readings);//this calculates the rotation angle of the servo
    runMPU();//collect data about position from the MPU
    rotationDetection();//determines if the user rotated from their original position
    timer();//refresh rate of the whole syste,
}

//=======================================================================================================================

//this pings the ultrasonic ang gathers the distances of objects
void pingUltrasonic()
{
    /* Trigger the sensor by sending a HIGH pulse of 10 or more microseconds */
    digitalWriteFast(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWriteFast(trigPin, LOW);
  
    duration_us = pulseIn(echoPin, HIGH);
    
    /* Convert the time into a distance */
    // Sound travels at 1130 ft/s (73.746 us/inch)
    // or 340 m/s (29 us/cm), out and back so divide by 2
    // Ref: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
    float inches = (float)duration_us / 74 / 2;
    
    readings[readingsIndex++] = inches;
    readingsIndex = readingsIndex % readingsLen;
}

// determines the driection and angle of the servo motor
int rotateServo(int rotationChange){
    static int angleOfServo = 0;
    static bool forward = true;
    
    angleOfServo += ((forward ? 1 : -1) * servoSpeed); // smooth rotation TODO: make it so it rotates smoothly 
    
    if(angleOfServo < rotationChange){
        forward = true;
    }
    else if(angleOfServo > rotationChange){
        forward = false;
    }
    
    if(angleOfServo < 0){
        angleOfServo = 0;
        forward = true;
    }
    else if(angleOfServo > 180){
        angleOfServo = 180;
        forward = false;
    }
    
    servoMotor.write(rotationChange);
    return angleOfServo;
}

//performs the calculations for the servo motor
void calculateServoRotation(float* readings){
    
    float inches = 0;
    for(int i = 0; i < readingsLen; i++) {
        inches += readings[i];
    }
    
    inches /= readingsLen;
    
    Serial.printlnf("Inches: %.2f", inches);
    
     int maxDistance = buffer_distance;
     int minDistance = activation_distance;
    
    int servoAngle = 0;    
    
    if(inches - arm_length <= maxDistance) {
        servoAngle = servoRotationMax;
    }
    else if(inches - arm_length <= minDistance) {
        servoAngle = (int)(servoRotationMax * (1 - ((inches - arm_length) - maxDistance) / (float)(minDistance - maxDistance)));
    }
    rotateServo(servoAngle);
}

//checks if the button is pressed
void checkButton(){
    static int holdTime = 0;  //tracks the length of time in which the user holds down the button
    static bool buzzTime = false; //boolean indicating if it is time to activate the buzzer (time to buzz)
    static bool pressed = false; // used to track if the button is currently pressed. set to false once button is released
    
    //if the button is pressed
    if(digitalRead(buttonpin) == LOW){
        pressed = true;
        Serial.println("button pressed");
        float inches = 0;
        for(int i = 0; i < readingsLen; i++) {
           inches += readings[i];
        }
        inches /= readingsLen;
        //Serial.printlnf("inches: %.2f", inches);
        activation_distance = inches;
        if(arm_length >= activation_distance){
            buffer_distance = 0;
        }
        else{
            buffer_distance = 2;
        }
        
        if(inches <= arm_length){
            Serial.println("buzz time true");
            buzzTime = true;
        }
        holdTime++;
        Serial.println(holdTime);
        
        startOrientation = ypr[0] * 180/M_PI;
    }
    else{
        pressed = false;
    }


    if(buzzTime && !pressed){
        Serial.println("triple ring");
        digitalWrite(buzzerpin, LOW);
        delay(100);
        digitalWrite(buzzerpin, HIGH);
        delay(500);
        buzzTime = false;
        pressed = false;
    }
    
}

//perfoms the calculations fo the MPU. This was code that from an example in the MPU library. most of it is copied so there is more functionality here than what
//is used when actually using the device
void runMPU(){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if ((mpuIntStatus & 0x02) > 0) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

    }
}

//simple function to detect over rotation of the user
void rotationDetection(){
    
    //TODO: rmemeber to make the buzzer sound while the MPU is setting itself
    
    float yaw = ypr[0] * 180/M_PI;
    static float rotationThreshold = 85;
    static float rotationMax = 179;
    
    //how this works
    //take the forward position set by the button
    //  no need to call to check the button in this, it should already be set
    //calculate the change in direction based on the absolute value of starting direction - current direction
    //if the result of the previous calculation is > 84, turn on the buzzer.
    //  since we are using the active buzzer, one buzz = turn left, two buzz = turn right
    //complete buzz efect before rechecking
    
    if(startOrientation != NULL){
        float yDif = fabs(startOrientation - yaw);
        Serial.println(yDif);
        if(yDif > rotationThreshold){
            digitalWrite(buzzerpin, LOW);
            delay(100);
            digitalWrite(buzzerpin, HIGH);
        }
    }
    
}

//this function exist to keep an internal timer and perform sudo multi processing.
//The MPU encounters FIFO overflow when delays are used. this circumvents the need to use delays by keeping track of the current loop and when a delay would have started
//as of now, this code is not used, but may be used later for greater funcitonality
void timer(){ 
    systemTimer++;
    Serial.printlnf("SystemTimer: %d", systemTimer);
}




