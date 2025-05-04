#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Wire.h>

// The current setup is using SPI
#define SDA_PIN 4
#define SCL_PIN 5
#define BNO08X_INT 6  // Use a free GPIO, not SDA/SCL
#define BNO08X_RST 7   // Add reset pin (connect to BNO08x RST)
#define BNO08X_ADDR 0x4A

unsigned long previousDebugMillis = 0;
#define DEBUG_INTERVAL_MILLISECONDS 30


BNO08x imu;

// timestamps
uint64_t timeStamp;

// accels
float ax;
float ay;
float az;

// gyros
float gx;
float gy;
float gz;

// mags
float mx;
float my;
float mz;

// quat
float quatI;
float quatJ;
float quatK;
float quatReal;
float quatRadianAccuracy;

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");

  if (imu.enableAccelerometer(1) == true) {
    Serial.println(F("Accelerometer enabled")); // Output in form x, y, z, in m/s^2
  } else {
    Serial.println("Could not enable accelerometer");
  }

  if (imu.enableGyro(1) == true) {
    Serial.println(F("Gyro enabled")); // Output in form x, y, z, in radians per second
  } else {
    Serial.println("Could not enable gyro");
  }

  if (imu.enableMagnetometer(1) == true) {
    Serial.println(F("Magnetometer enabled")); // Output in form x, y, z, in uTesla
  } else {
    Serial.println("Could not enable magnetometer");
  }

  if (imu.enableRotationVector(1) == true) {
    Serial.println(F("Rotation vector enabled")); // Output in form i, j, k, real, accuracy
  } else {
    Serial.println("Could not enable rotation vector");
  }

  delay(100);
}


void setup() {
  // Specify I2C / SPI pins
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  
  // Begin
  Wire.begin();
  Serial.begin(115200);
  while (!Serial) delay(10); // wait until Serial becomes avaliable

  // Hardware reset sequence
  pinMode(BNO08X_RST, OUTPUT);
  digitalWrite(BNO08X_RST, LOW);
  delay(50);
  digitalWrite(BNO08X_RST, HIGH);
  delay(500);

  if (!imu.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("IMU not found!");
    while(1);
  }
  Serial.println("BNO08x found!");

  setReports();

}


void loop() {

  if (imu.wasReset()) {
    Serial.println("sensor was reset");
    setReports();
  }

  if (imu.getSensorEvent() == true) {

    uint8_t reportID = imu.getSensorEventID();
    switch (reportID)
    {
    case SENSOR_REPORTID_ACCELEROMETER:
      ax = imu.getAccelX();
      ay = imu.getAccelY();
      az = imu.getAccelZ();
      break;
    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
      gx = imu.getGyroX();
      gy = imu.getGyroY();
      gz = imu.getGyroZ();
      break;
    case SENSOR_REPORTID_MAGNETIC_FIELD:
      mx = imu.getMagX();
      my = imu.getMagY();
      mz = imu.getMagZ();
      break;
    case SENSOR_REPORTID_ROTATION_VECTOR:
      timeStamp = imu.getTimeStamp();
      quatI = imu.getQuatI();
      quatJ = imu.getQuatJ();
      quatK = imu.getQuatK();
      quatReal = imu.getQuatReal();
      quatRadianAccuracy = imu.getQuatRadianAccuracy();
    default:
      break;
    }

    // time since last debug data printed to terminal
    int timeSinceLastSerialPrint = (millis() - previousDebugMillis);

    // Only print data to the terminal at a user deficed interval
    if(timeSinceLastSerialPrint > DEBUG_INTERVAL_MILLISECONDS)
    {
      Serial.print(ax);
      Serial.print(F(","));
      Serial.print(ay);
      Serial.print(F(","));
      Serial.print(az);
      Serial.print(F(","));

      Serial.print(gx);
      Serial.print(F(","));
      Serial.print(gy);
      Serial.print(F(","));
      Serial.print(gz);
      Serial.print(F(","));

      Serial.print(mx);
      Serial.print(F(","));
      Serial.print(my);
      Serial.print(F(","));
      Serial.print(mz);
      Serial.print(F(","));

      Serial.print(quatI);
      Serial.print(F(","));
      Serial.print(quatJ);
      Serial.print(F(","));
      Serial.print(quatK);
      Serial.print(F(","));
      Serial.print(quatReal);
      Serial.print(F(","));
      Serial.print(quatRadianAccuracy);
      Serial.print(F(","));
      Serial.print(timeStamp);
      Serial.print(F(","));

      Serial.print(timeSinceLastSerialPrint);

      Serial.println();

      previousDebugMillis = millis();

    }


  }

  // delay(1);
}

// void setup() {
//   delay(2000);
//   Serial.begin(115200);
//   while (!Serial) delay(10);
//   Serial.println("Hello, RP2040!");
// }
// void loop() {
//   Serial.println("Hi");
//   delay(1000);
// }