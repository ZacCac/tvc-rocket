/* used to figure out wire.h library:     https://forum.arduino.cc/index.php?topic=563091.0 */
/* BNO Github:                            https://github.com/arduino-libraries/BNO055 */
/* Rocket PID Sim:                        https://codesandbox.io/s/rocket-sim-final-0oq6u?file=/src/index.js */

#include <Wire.h>

const int BNO_ADDR = 0X28;
const int PWR_MODE = 0x3E;
const int NORMAL = 0x00;                                // sleep = 0x01, suspend = 0x10, force = 0x11
const int OPR_MODE = 0x3D;                              // operation mode
const int AMG = 0x07;                                   // all sensors active, fusion = 0x0B

float accx, accy, accz;
float gyrox, gyroy, gyroz;
float magx, magy, magz;

// TIMER VARIABLES
long current_millis;
long read_millis = 0;
long read_interval = 100;
long serial_millis = 0;
long serial_interval = 100;


void setup() {
  Wire.begin();
  Serial.begin(112500);
  while(!Serial) delay(10);

  initialize_bno();
  //gyro_offsets();       // not working atm, dont know if I even need it

  //while(1);             // testing only
}

void loop() {
  current_millis = millis();

  if (current_millis - read_millis > read_interval) {
    data_read();
    read_millis = current_millis;
  }
  if (current_millis - serial_millis > serial_interval){
    serial_out();
    serial_millis = current_millis;
  }
}

void initialize_bno() {
  // POWER MODE
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(PWR_MODE);
  Wire.write(NORMAL);
  Wire.endTransmission();

  delay(100);                                     // reduce time later?

  // CONFIG MODE
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(OPR_MODE);
  Wire.write(0x00);                               // puts BNO into CONFIG mode
  Wire.endTransmission();

  // REMAPPING AXES
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x41);
  Wire.write(0x18);                               // z-vertical, y-front x- horizontal      default: 0x24
  Wire.endTransmission();

  // OPERATION MODE
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(OPR_MODE);
  Wire.write(AMG);                                // puts BNO into AMG mode (activates all sensors)
  Wire.endTransmission();
}

void gyro_offsets() {
  float offsetx = 0, offsety = 0, offsetz = 0;

  Serial.print("Calculating gyro offsets");

  // iterates 10 times and averages the data to calculate the gyro offsets
  for (int i = 0; i < 100; i++)
  {
    Wire.beginTransmission(BNO_ADDR);
    Wire.write(0x14);
    Wire.endTransmission(false);
    Wire.requestFrom(BNO_ADDR,6,true);

    offsetx+=(int16_t)(Wire.read()|Wire.read()<<8) /16.0; // Dps
    offsety+=(int16_t)(Wire.read()|Wire.read()<<8) /16.0; // Dps
    offsetz+=(int16_t)(Wire.read()|Wire.read()<<8) /16.0; // Dps

    Wire.endTransmission();

    if (i%10 == 0) Serial.print("."); // progress bar

    delay(100);
  }
  Serial.println();

  // offset calc
  offsetx = offsetx / 6.25;     // (offsetx / 100 * 16)
  offsety = offsety / 6.25;
  offsetz = offsetz / 6.25;

  Serial.print(offsetx);
  Serial.print(",");
  Serial.print(offsety);
  Serial.print(",");
  Serial.print(offsetz);
  Serial.println();

  int test = offsetx;

  // CONFIG MODE
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(OPR_MODE);
  Wire.write(0x00);                   // puts BNO into CONFIG mode
  Wire.endTransmission();

  // STORING OFFSETS
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x61);                   // 0x61 or 0x62
  Wire.write(test);
  Wire.endTransmission();

  // OPERATION MODE
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(OPR_MODE);
  Wire.write(AMG);                    // puts BNO into AMG mode (activates all sensors)
  Wire.endTransmission();
}

void data_read() {
  Wire.beginTransmission(BNO_ADDR);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(BNO_ADDR,18,true);

  accx=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  accy=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2
  accz=(int16_t)(Wire.read()|Wire.read()<<8 )/100.00; // m/s^2

  magx=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // mT
  magy=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // mT
  magz=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // mT

  gyrox=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  gyroy=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps
  gyroz=(int16_t)(Wire.read()|Wire.read()<<8 )/16.00; // Dps

  Wire.endTransmission();
}

void serial_out() {
  /*
  Serial.print(accx);
  Serial.print(",");
  Serial.print(accy);
  Serial.print(",");
  Serial.print(accz);

  Serial.print(",");
  */
  Serial.print(gyrox);
  Serial.print(",");
  Serial.print(gyroy);
  Serial.print(",");
  Serial.print(gyroz);
  /*
  Serial.print(",");

  Serial.print(magx);
  Serial.print(",");
  Serial.print(magy);
  Serial.print(",");
  Serial.print(magz);
  */
  Serial.println();
}
