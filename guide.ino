#include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <ESP32Servo.h>
#include <7Semi_ICM20948.h>
#include <MadgwickAHRS.h>
#include <PID_v1.h>
#include <controller.h>
// ===================== Pins & Constants =====================
#define RX_PIN 16
#define TX_PIN 17
#define SERVOPIN 26
#define SDA 21
#define SCL 22
#define ICM_ADDR 0x69
#define LEFT_SERV
#define RIGHT_SERV

static const uint32_t GPSBaud = 9600;
static const unsigned long PRINT_INTERVAL_MS = 1000;

//EXAMPLE TARGET USED FOR TESTING
static const double GEISEL_LAT = 32.88116;
static const double GEISEL_LON = -117.237651;

//WILL NEED TO UPLOAD TARGET DATA FROM WIFI, SEE PAYLOAD V1 (left as GEISEL LOCATION FOR TESTING)
static double TARGET_LAT = GEISEL_LAT;
static double TARGET_LON = GEISEL_LON;

// ===================== Objects =====================
TinyGPSPlus gps;
Servo serv;
ICM20948_7Semi imu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Madgwick orientation;

// ===================== Timing =====================
unsigned long lastPrintTime = 0;
unsigned long calibrationTime = 5000;
// ===================== Function Prototypes =====================
void readGPS();
bool hasGPSFix();
float calculateTurnAngle(double currLat, double currLon, float facing);
void readIMU(float &gx, float &gy, float &gz, float &ax, float &ay, float &az);
float getHeadingDegrees(double roll, double pitch);
void printOrientation();
void printGPSData(double lat, double lon, unsigned long distance);
void printIMUData(float gx, float gy, float gz);
void printTurnAngle(float turnAngle);
float wrapAngle180(float angle);

// ===================== Setup =====================
void setup(void) {
  Serial.begin(19200);
  Serial1.begin(GPSBaud, SERIAL_8N1, RX_PIN, TX_PIN);

  orientation.begin(500);
  //serv.attach(SERVOPIN);
  //serv.setPeriodHertz(50);
  //serv.write(90);

  Wire.begin(SDA, SCL);

  if (!imu.begin(Wire, ICM_ADDR)) {
    Serial.println(F("ERROR: begin(I2C) failed"));
    while (1) delay(200);
  }

  if (!imu.applyBasicDefaults()) {
    Serial.println(F("ERROR: applyBasicDefaults() failed."));
    while (1) delay(200);
  }

  if (!mag.begin()) {
    Serial.println("No HMC5883 detected");
    while (1)
      ;
  }
  if (!imu.AccelConfigure(ACCEL_DLPFCFG_3, /*FS_SEL*/ g4,
                          /*dlpf_enable*/ true,
                          /*dec3*/ ACCEL_DEC3_AVG_8,
                          /*stX*/ false, /*stY*/ false, /*stZ*/ false)) {
    Serial.println(F("AccelConfigure failed."));
    while (1)
      ;
  }

  Serial.println("System Initialized");
}

// ===================== Loop =====================
void loop(void) {
  readGPS();
  sensors_event_t event;
  mag.getEvent(&event);
  static bool calibrated = false;
  
  if (!gps.location.isValid()) {
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
      lastPrintTime = millis();
      Serial.println("Waiting for GPS fix...");
    }
    if(millis() > 3000){
      calibrationTime = millis();
    }
    return;
  }

  // ----- GPS Data -----
  double startLat = gps.location.lat();
  double startLon = gps.location.lng();
  delay(5);
  double currLat = gps.location.lat();
  double currLon = gps.location.lng();

  // ---- Get Velocity ----
  double velocity = distanceBetween(start lat, startLon, currLat, currLon) / 5;
  
  unsigned long distanceToTarget =
    (unsigned long)TinyGPSPlus::distanceBetween(
      currLat, currLon, TARGET_LAT, TARGET_LON);
  // ----- IMU Data -----
  float gx, gy, gz;
  float ax, ay, az;
  //float mx, my, mz;
  readIMU(gx, gy, gz, ax, ay, az);

  // ---- Calibrating IMU Yaw ----
  orientation.updateIMU(
    (gx*PI)/180, (gy*PI)/180, (gz*PI)/180,
    ax, ay, az/*, //30:247,
    event.magnetic.x, -event.magnetic.y, event.magnetic.z*/);
  float headingDegrees = getHeadingDegrees(0,0);//orientation.getRoll(), -orientation.getPitch());

  if (!calibrated && millis() - calibrationTime < 5000) {
    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
      Serial.println("Keep IMU and GPS aligned and flat for calibration");
    }
    return;
  }

  if (!calibrated && millis() >= calibrationTime) {
    calibrated = true;
    Serial.println("Calibration complete");
  }
  //NOTE THIS WAS USED FOR CALIBRATING DRIFT, COMMENT OUT TIL NEXT COMMENT WHEN NOT CALIBRATING
  /*
  static unsigned long count = 0;
  static float gxt = 0;
  static float gyt = 0;
  static float gzt = 0;
  gxt += gx;
  gyt += gy;
  gzt += gz;
  count++;
  Serial.printf("Gx avg: %f\tGy avg: %f\tGz avg: %f\n",
              gxt / count,
              gyt / count,
              gzt / count);
  */
  //END COMMENT HERE
  //float facing2 = (orientation.getYaw() - yawOffset);
  float facing = headingDegrees;
  float turnAngle = calculateTurnAngle(currLat, currLon, facing);

  // ----- Timed Printing -----
  if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = millis();

    printGPSData(currLat, currLon, distanceToTarget);
    //printIMUData(gx, gy, gz);
    double courseToTarget =
    TinyGPSPlus::courseTo(currLat, currLon, TARGET_LAT, TARGET_LON);
    printf("Heading: %f\n", headingDegrees);
    printf("course to: %f\n", courseToTarget);
    printOrientation();
    //printf("yawoffset: %f\n", yawOffset);
    //printf("Facing2: %f\n", facing2);
    printTurnAngle(turnAngle);  
    }
  delay(2);  // keep loop responsive
}

// ===================== Helper Functions =====================

void readGPS() {
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
}

//On start Yaw = 150, Heading = 120
float calculateTurnAngle(double currLat, double currLon, float facing) {
  double courseToTarget =
    TinyGPSPlus::courseTo(currLat, currLon, TARGET_LAT, TARGET_LON);
  //printf("course to: %f\n", courseToTarget);
  float turnAngle = wrapAngle180(courseToTarget - facing);
  return turnAngle;

void readIMU(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
  if (!imu.readGyro(gx, gy, gz)) {
    Serial.println(F("GYR read failed"));
  }
  if (!imu.readAccel(ax, ay, az)) {
    Serial.println(F("ACC read failed"));
  }
  /*
  if(!imu.readMag(mx, my, mz))
  {
    Serial.printf("Mag read failed\n");
  }
  */

  //Used to offset IMU drift bias (ignore my scuffed calibration tactics)
  gx += -.3485 + .036 + .028 - .11 + .023 + .041 -.025;
  gy += .622 - .0888 + .058 - .031 -.044 -.01;
  gz += -.2905 + .0384 - .039 + .039 -.013 +.01;
}

//gets the heading using declination angle, must be updated when location changes use:
//https://www.magnetic-declination.com/
float getHeadingDegrees(double roll, double pitch) {
  sensors_event_t event;
  mag.getEvent(&event);
  //Note: obtained offsets through testing
  float calMx = event.magnetic.x + 2.13;
  float calMy = event.magnetic.y + 11.27;
  float calMz = event.magnetic.z -5.91;

  double Xh = calMx * cos(radians(pitch)) + calMz * sin(radians(pitch));

  double Yh = calMx * sin(radians(roll)) * sin(radians(pitch))
   + calMy * cos(radians(roll))
   + calMz * sin(radians(roll)) * cos(radians(pitch));

  float heading = atan2(-Yh, Xh);

  //NOTE DECLINATION ANGLE MUST BE CHANGED BASED ON LOCATION FOR SCHOOL THE NUMBER IS -.174
  float declinationAngle = -.174;
  //The second numbers comes from the offset the GPS needs to have the logo face the same way as the way the payload is facing
  float logoOffset = radians(31);
  heading += declinationAngle + logoOffset;

  heading = fmod(heading + 2*PI, 2*PI);


  return heading * 180.0 / PI;
}

//Method to keep angle within + or - 180
float wrapAngle180(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}


void printGPSData(double lat, double lon, unsigned long distance) {
  Serial.print("Lat: ");
  Serial.println(lat, 6);
  Serial.print("Lon: ");
  Serial.println(lon, 6);
  Serial.print("Distance to Geisel (m): ");
  Serial.println(distance);
}

void printOrientation() {
  //*
  Serial.printf("Roll: %.2f Pitch: %.2f Yaw: %.2f\n",
                orientation.getRoll(),
                -orientation.getPitch(),
                orientation.getYaw());
  //             */
  //Serial.printf("Pitch: %.2f\n", orientation.getPitch());
}

void printIMUData(float gx, float gy, float gz) {
  Serial.print("Gx: ");
  Serial.print(gx);
  Serial.print("\tGy: ");
  Serial.print(gy);
  Serial.print("\tGz: ");
  Serial.println(gz);
}

void printTurnAngle(float turnAngle) {
  if (turnAngle <= 5 && turnAngle >= -5) {
    Serial.println("You are facing the Target!");
  } else if (turnAngle > 5) {
    Serial.print("Turn right ");
    Serial.print(turnAngle, 1);
    Serial.println(" degrees");
  } else {
    Serial.print("Turn left ");
    Serial.print(-turnAngle, 1);
    Serial.println(" degrees");
  }
}
