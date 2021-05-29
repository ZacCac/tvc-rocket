#include <Wire.h>
#include <math.h>

class Bno055 
{
private:
    struct Accelerometer {
        float x, y, z;  // m/s^2
    };
    struct Magnetometer {
        float x, y, z;  // mT
    };
    struct Gyroscope {
        double x, y, z;  // dps (degrees per second)
    };
    struct Quaternion {
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;   // initial conditions
        double wdot, xdot, ydot, zdot;
    };

    const int bnoAddress_ = 0x28;
    const int powerMode_ = 0x3E;
    const int normal_ = 0x00;
    const int oprMode_ = 0x3D;
    const int amg_ = 0x07;
    const int sysTrigger_ = 0x3F;

    float angle;

    // initializing structs
    Accelerometer a;
    Magnetometer m;
    Gyroscope g;
    Quaternion q;

public:
    // Initializes the BNO055 Sensor and remaps axes to the correct orientation
    void InitializeBno() 
    {
        // POWER MODE
        Wire.beginTransmission(bnoAddress_);
        Wire.write(powerMode_);
        Wire.write(normal_);
        Wire.endTransmission();
        delay(100);             // reduce time later?

        // CONFIG MODE (3.3 Operation Modes)
        Wire.beginTransmission(bnoAddress_);
        Wire.write(oprMode_);
        Wire.write(0x00);       // puts BNO into CONFIG mode
        Wire.endTransmission();

        // REMAPPING AXES (3.4 Axis Remap)
        Wire.beginTransmission(bnoAddress_);
        Wire.write(0x41);
        Wire.write(0x18);       // z-vertical, y-front x- horizontal      default: 0x24
        Wire.endTransmission();

        // SETTING EXTERNAL CRYSTAL (4.3.63 SYS_TRIGGER)
        Wire.beginTransmission(bnoAddress_);
        Wire.write(sysTrigger_);
        Wire.write(0x80);
        Wire.endTransmission();

        // OPERATION MODE (3.3 Operation Modes)
        Wire.beginTransmission(bnoAddress_);
        Wire.write(oprMode_);
        Wire.write(amg_);        // puts BNO into AMG mode (activates all sensors)
        Wire.endTransmission();

        Serial.println("Bno055 Initialized");
    }

    // Gets the raw data from the BNO055 Sensor and puts the data in their respective arrays
    void getBnoData() 
    {
        // Starts transmission with the sensor
        Wire.beginTransmission(bnoAddress_);
        Wire.write(0x08);
        Wire.endTransmission(false);
        Wire.requestFrom(bnoAddress_,18,true);

        // Puts raw data into structs

        a.x = (int16_t)(Wire.read()|Wire.read()<<8 )/100.00;  // m/s^2
        a.y = (int16_t)(Wire.read()|Wire.read()<<8 )/100.00;  // m/s^2
        a.z = (int16_t)(Wire.read()|Wire.read()<<8 )/100.00;  // m/s^2

        m.x = (int16_t)(Wire.read()|Wire.read()<<8 )/16.00;   // mT
        m.y = (int16_t)(Wire.read()|Wire.read()<<8 )/16.00;   // mT
        m.z = (int16_t)(Wire.read()|Wire.read()<<8 )/16.00;   // mT
        
        g.x = (int16_t)(Wire.read()|Wire.read()<<8 )/16.00;   // Dps
        g.y = (int16_t)(Wire.read()|Wire.read()<<8 )/16.00;   // Dps
        g.z = (int16_t)(Wire.read()|Wire.read()<<8 )/16.00;   // Dps

        // End transmission with the sensor
        Wire.endTransmission();
    }

    // Calculates the quaternion position, dt is the sampling time in seconds
    void getQuaternions(double dt)
    {
        getBnoData(); // need gyro reading for quaternion calculations
        
        // temp variables, represent half of each component of the quaternion vector
        double halfW = 0.5 * q.w;
        double halfX = 0.5 * q.x;
        double halfY = 0.5 * q.y;
        double halfZ = 0.5 * q.z;

        double norm = 0.0;    // used to normalize the quaternion (unit vector)

        angle += g.x * dt;

        // quaternion derivatives, math derived elsewhere
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
        //Serial.print(norm, 9);
        //Serial.print(" ");
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
        Serial.print(angle);
        
        Serial.println();

        

       
    }
};

Bno055 bno;                 // creating object bno

long pMillis = 0;           // ms (milliseconds)
double interval = 100.0;     // ms (milliseconds)
int i = 0;

void setup() 
{
  Wire.begin();
  Serial.begin(112500);
  delay(10);

  bno.InitializeBno();
  delay(100);
  
}

void loop() 
{
    long cMillis = millis();
    
    if (cMillis - pMillis >= interval && i < 1000) {    // will run every interval
        bno.getQuaternions(interval / 1000.0);
        pMillis = cMillis;
        i++;
    }
    
    
}
