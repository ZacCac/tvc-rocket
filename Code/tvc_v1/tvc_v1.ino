#include <Wire.h>
#include <math.h>
#include <Servo.h>

struct Xyz {
    double x = 0, y = 0, z = 0;
};

long pMillis = 0;                       // previous time in ms (milliseconds)
double interval = 10.0;                 // sampling interval in ms (milliseconds)
uint8_t xPin = 22;                      // pin for the TVC x-axis servo
uint8_t yPin = 21;                      // pin for the TVC y-axis servo
uint8_t pPin = 9;                       // pin for the parachute servo
uint8_t button = 15;
int buttonState = 0;

double xServoOffset = 90.0;             // offset of the TVC y-axis servo
double yServoOffset = 90.0;             // offset of the TVC y-axis servo
int pServoPos = 90;                     // starting position of the parachute servo



/*###############################################################################################*/

class Bno055 
{
private:
/*
    // Data Structs
    struct Xyz {                        // Struct for three axes
        double x = 0, y = 0, z = 0;
    };
*/
    struct Quaternion {                 // Quaternion Struct
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;   // initial conditions
        double wdot, xdot, ydot, zdot;
    };

    // initializing structs
    Xyz a;                              // Accelerometer
    Xyz m;                              // Magnetometer
    Xyz g;                              // Gyroscope
    Xyz e;                              // Euler Angles
    Xyz g_o;                            // Gyroscope offsets
    Quaternion q;                       // Quaternions

    //https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
    const int BNO_ADDR = 0x28;          // I2C Address of the sensor                                            MAY CHANGE WITH MULTIPLE SENSORS

    const int PWR_MODE = 0x3E;          // Power mode,default normal                more info section 3.2
    const int NORMAL = 0x00;            // all required sensors are always on

    const int OPR_MODE = 0x3D;          // Change operating mode                    more info section 3.3
    const int CONFIGMODE = 0x00;        // Config mode... nothing else
    const int AMG = 0x07;               // Sensor actively measuring

    const int AXIS_MAP_CONFIG = 0x41;   // remap axes, default 0x24                 more info section 3.4
    const int yz_remap_ = 0x18;         // z: up/down y: forward/backward x: right/left

    const int AXIS_MAP_SIGN = 0x42;     //                                                                      CHECK AXIS SIGNS, MAY NOT BE NEEDED

    const int SYS_TRIGGER = 0x3F;       // Used to set clock, reset, int
    const int CLK_SEL = 0x80;           // sets external crystal clock

    const int ACC_DATA_X_LSB = 0x08;    // Lower byte of X axis acc data
    const int ACC_Config = 0x08;        // used to set range, default 0x0D
    const int ACC_RANGE = 0x11;         // can only be changed in sensor mode       more info section 3.5.2

    const int GYR_DATA_X_LSB = 0x14;    // Lower byte of X axis gyr data
    const int GYR_Config_0 = 0x0A;      // used to set range/bandwidth
    const int GYR_RANGE = 0x11;         // 1000 dps, default 2000 dps & 32 Hz       more info section 3.5.3

    const int UNIT_SEL = 0x3B;          // Change units of measurements
    const int dps_to_rps_ = 0x02;       // changes units of gyr to rad/s            more info section 3.6.1

    const int MAG_DATA_X_LSB = 0x0E;    // Lower byte of X axis mag data

public:
    // Initializes the BNO055 Sensor by setting up the specific configurations required, remapping axes, and preparing the sensor to read data
    void InitializeBno() 
    {
        char data[2];                       // temp variable to hold the addresses
        int offset_iterations = 1000;        // how many iterations will be used to find the average offset of the sensor

        // Resetting the power and operation modes back to default in case they were switched
        data[0] = PWR_MODE; data[1] = NORMAL;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);
        data[0] = OPR_MODE; data[1] = CONFIGMODE;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        // Remapping axes
        data[0] = AXIS_MAP_CONFIG; data[1] = yz_remap_;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        // Setting external crystal use
        data[0] = SYS_TRIGGER; data[1] = CLK_SEL;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        // Adjusting accelerometer config
        data[0] = ACC_Config; data[1] = ACC_RANGE;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        // Adjusting gyroscope config
        data[0] = GYR_Config_0; data[1] = GYR_RANGE;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        // Changing gyroscope units to rad/s
        data[0] = UNIT_SEL; data[1] = dps_to_rps_;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        // Changing operating mode to AMG
        data[0] = OPR_MODE; data[1] = AMG;
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(data, 2);
        Wire.endTransmission(true); delay(20);

        Serial.println("BNO055 Initialized");
        Serial.print("Calibrating gyro, do not move");

        // Summing up 100 gyroscope readings
        for (int i = 0; i <= offset_iterations; i++)
        {
            Wire.beginTransmission(BNO_ADDR);
            Wire.write(GYR_DATA_X_LSB);
            Wire.endTransmission(false);
            Wire.requestFrom(BNO_ADDR, 6, true);

            g_o.x += (int16_t)(Wire.read() | Wire.read()<<8);
            g_o.y += (int16_t)(Wire.read() | Wire.read()<<8);
            g_o.z += (int16_t)(Wire.read() | Wire.read()<<8);

            delay(interval/1000.0);

            // Progress Bar
            
            if(i % 100 == 0) 
            {
                Serial.print(".");
            }
            
        }
        Serial.println();

        // Averaging the gyroscope drift
        g_o.x /= offset_iterations;
        g_o.y /= offset_iterations;
        g_o.z /= offset_iterations;

        // Reading out offsets
        Serial.print("Gyr X Offset: "); Serial.println(g_o.x);
        Serial.print("Gyr Y Offset: "); Serial.println(g_o.y);
        Serial.print("Gyr Z Offset: "); Serial.println(g_o.z);
    }

    // Gets the raw data from the BNO055 Sensor
    void getBnoData()
    {
        // Starts transmission with the sensor
        Wire.beginTransmission(BNO_ADDR);
        Wire.write(ACC_DATA_X_LSB);
        Wire.endTransmission(false);
        Wire.requestFrom(BNO_ADDR, 18, true);

        // Puts raw data into structs
        a.x = (Wire.read() | Wire.read()<<8 ) / 100.00;  // m/s^2
        a.y = (Wire.read() | Wire.read()<<8 ) / 100.00;  // m/s^2
        a.z = (Wire.read() | Wire.read()<<8 ) / 100.00;  // m/s^2

        m.x = (Wire.read() | Wire.read()<<8) / 16.00;   // mT
        m.y = (Wire.read() | Wire.read()<<8) / 16.00;   // mT
        m.z = (Wire.read() | Wire.read()<<8) / 16.00;   // mT
        
        g.x = ((int16_t)(Wire.read() | Wire.read()<<8) - g_o.x) / 900.00 ;   // rad/s
        g.y = ((int16_t)(Wire.read() | Wire.read()<<8) - g_o.y) / 900.00 ;   // rad/s
        g.z = ((int16_t)(Wire.read() | Wire.read()<<8) - g_o.z) / 900.00 ;   // rad/s

        // End transmission with the sensor
        Wire.endTransmission();

        
        Serial.print(a.x, 9);
        Serial.print(" ");
        Serial.print(a.y, 9);
        Serial.print(" ");
        Serial.print(a.z, 9);
        Serial.print(" ");
        Serial.print(m.x, 9);
        Serial.print(" ");
        Serial.print(m.y, 9);
        Serial.print(" ");
        Serial.print(m.z, 9);
        Serial.print(" ");
        Serial.print(g.x, 9);
        Serial.print(" ");
        Serial.print(g.y, 9);
        Serial.print(" ");
        Serial.print(g.z, 9);
        Serial.print(" ");    
    }

    // Calculates the quaternion position, dt is the sampling time in seconds
    void getQuaternions(double dt)
    {
        getBnoData(); // need gyro reading for quaternion calculations, within another function
        
        // temp variables, represent half of each component of the quaternion vector
        double halfW = 0.5 * q.w;
        double halfX = 0.5 * q.x;
        double halfY = 0.5 * q.y;
        double halfZ = 0.5 * q.z;

        double norm = 0.0;    // used to normalize the quaternion (unit vector)

        // quaternion derivatives, math derivation: https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf
        q.wdot = -halfX * g.x - halfY * g.y - halfZ * g.z; // w
        q.xdot =  halfW * g.x + halfY * g.z - halfZ * g.y; // x
        q.ydot =  halfW * g.y - halfX * g.z + halfZ * g.x; // y
        q.zdot =  halfW * g.z + halfX * g.y - halfY * g.x; // z

        // integrating quaternions
        q.w += q.wdot * dt;
        q.x += q.xdot * dt;
        q.y += q.ydot * dt;
        q.z += q.zdot * dt;

        norm = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z; 
        norm = sqrt(norm);

        q.w = q.w / norm;
        q.x = q.x / norm;
        q.y = q.y / norm;
        q.z = q.z / norm;

        Serial.print(dt, 9);
        Serial.print(" ");
        Serial.print(q.wdot, 9);
        Serial.print(" ");
        Serial.print(q.xdot, 9);
        Serial.print(" ");
        Serial.print(q.ydot, 9);
        Serial.print(" ");
        Serial.print(q.zdot, 9);
        Serial.print(" ");
        Serial.print(q.w, 9);
        Serial.print(" ");
        Serial.print(q.x, 9);
        Serial.print(" ");
        Serial.print(q.y, 9);
        Serial.print(" ");
        Serial.print(q.z, 9);
        Serial.print(" ");
        Serial.print(norm, 9);
        Serial.print(" ");  
    }

    // Converts the orientation from quaternions to euler angles
    struct Xyz quatToEuler(double dt)
    {
        getQuaternions(dt);

        // x-axis
        double temp1 = 2 * (q.w * q.x + q.y * q.z);
        double temp2 = 1 - 2 * (q.x * q.x + q.y * q.y);
        e.x = atan2(temp1, temp2) * 180 / M_PI;

        // y-axis
        temp1 = 2 * (q.w * q.y - q.z * q.x);
        if (abs(temp1) >= 1) e.y = copysign(90, temp1); // use 90 degrees if out of range
        else e.y = asin(temp1) * 180 / M_PI;

        // z-axis
        temp1 = 2 * (q.w * q.z + q.x * q.y);
        temp2 = 1 - 2 * (q.y * q.y + q.z * q.z);
        e.z = atan2(temp1, temp2) * 180 / M_PI;

        Serial.print(e.x);
        Serial.print(" ");
        Serial.print(e.y);
        Serial.print(" ");
        Serial.print(e.z);
        Serial.print(" ");  

        return e; 
    }
};

double tvcAngle(double);

Bno055 bno;                             // creating object bno
Servo xServo;                           // TVC x-axis servo
Servo yServo;                           // TVC y-axis servo
Servo pServo;                           // parachute servo
Xyz e;                                  // struct for Euler angles in TVC

/*###############################################################################################*/

void setup() 
{

    Wire.begin();
    Serial.begin(112500);
    delay(10);

    // linking the servos with their corresponding pins on the flight computer
    xServo.attach(xPin);                
    yServo.attach(yPin);
    pServo.attach(pPin);

    // aligning the TVC
    xServo.write(xServoOffset);         
    yServo.write(yServoOffset);
    pServo.write(pServoPos);

    pinMode(button, INPUT);

    Serial.println("Waiting for button press to start calibration");
    while(buttonState == LOW) buttonState = digitalRead(button);
    bno.InitializeBno();
    delay(100);
}

/*###############################################################################################*/

void loop() 
{
    long cMillis = millis();
    
    if (cMillis - pMillis >= interval) {    // will run every interval
        //bno.getBnoData();
        //bno.getQuaternions(interval / 1000.0);
        //e = bno.quatToEuler(interval / 1000.0);

        //xServo.write(tvcAngle(e.x, xServoOffset));
        //yServo.write(tvcAngle(e.y, yServoOffset));

        parachuteDeploy();

        Serial.println();
        pMillis = cMillis;
    }
}

/*###############################################################################################*/

double tvcAngle(double eulerAngle, double servoOffset)
{
    uint8_t axisScaleFactor = 3;
    
    int8_t maxAngle = 15;

    double servoAngle = servoOffset - (eulerAngle / axisScaleFactor);   // calculates the angle that the servo needs to move to

    // servo angles if the max angle is exceeded
    if (servoAngle > servoOffset + maxAngle) servoAngle = servoOffset + maxAngle;
    else if (servoAngle < servoOffset - maxAngle) servoAngle = servoOffset - maxAngle;

    Serial.print(eulerAngle);
    Serial.print(" ");
    Serial.print(servoAngle);
    Serial.print(" ");

    return servoAngle;
}

void parachuteDeploy()
{
    buttonState = digitalRead(button);
    if (buttonState == HIGH) 
    {
        pServo.write(120);          // activate
        Serial.println("ACTIVATED");
        Serial.println("ACTIVATED");
        Serial.println("ACTIVATED");

        delay(1000);
        pServo.write(pServoPos);
    } 
}