#include <BMI160Gen.h>
#include "quaternion.h"
#include "sensor_processing_lib.h"
#include "vector_3d.h"

#include <mySD.h>

File root;

const int i2c_addr = 0x69;
int16_t gx, gy, gz, ax, ay, az;         // raw values

unsigned long Start = 0;
float delta,wx,wy,wz;
euler_angles angles;
vector_ijk fused_vector;
Quaternion q_acc; 

float laura_angle;

void setup() {


  pinMode(2,OUTPUT);
  
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open

  Serial.print("Initializing SD card...");
  /* initialize SD library with Soft SPI pins, if using Hard SPI replace with this SD.begin()*/
  // SCK, MISO, MOSI ,SS
  if (!SD.begin(26, 14, 13, 27)) {
    Serial.println("initialization failed!");
    while(1);
  }
  Serial.println("initialization done.");

  // initialize device
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
  //BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */5);
  
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  
  BMI160.setAccelerometerRate(1600);
  BMI160.setGyroRate(1600);
  
  Serial.print("Gyro Rate: \t");
  Serial.print(BMI160.getGyroRate());
  Serial.println();
  Serial.print("Acc Rate: \t");
  Serial.print(BMI160.getAccelerometerRate());
  Serial.println();

  Serial.print("Gyro Range: \t");
  Serial.print(BMI160.getGyroRange());
  Serial.println();
  Serial.print("Acc Range: \t");
  Serial.print(BMI160.getAccelerometerRange());
  Serial.println();


  Start = micros(); 
  fused_vector = vector_3d_initialize(0.0,0.0,-1.0);
  q_acc = quaternion_initialize(1.0,0.0,0.0,0.0);

  digitalWrite(2,HIGH);
  
}

void loop() {

  
  wx = 0.0005326487*gx;
  wy = 0.0005326487*gy;
  wz = 0.0005326487*gz;

  delta = 0.000001*(micros()-Start);
  //  Serial.print("Time Delta: \t");
  //  Serial.print(delta,16);
  //  Serial.println();
  fused_vector = update_fused_vector(fused_vector,ax,ay,az,wx,wy,wz,delta);

  q_acc = quaternion_from_accelerometer(fused_vector.a,fused_vector.b,fused_vector.c);
  angles = quaternion_to_euler_angles(q_acc);
 

  if(angles.yaw > 0 && angles.pitch > 0)
    laura_angle = angles.pitch;
  else if(angles.yaw < 0 && angles.pitch > 0)
    laura_angle = 180 - angles.pitch;
  else if(angles.yaw > 0 && angles.pitch < 0)
    laura_angle = 180 - angles.pitch;    
  else if(angles.yaw < 0 && angles.pitch < 0)
    laura_angle = 360 + angles.pitch;
  else;
    //Serial.println("Quadrant Calculator ERRROR");
  



  Start = micros();

  /* Begin at the root "/" */
  root = SD.open("/");
  if (root) {    
    printDirectory(root, 0);
    root.close();
  } else {
    Serial.println("error opening test.txt");
  }


  /* open "test.txt" for writing */
  root = SD.open("test.txt", FILE_WRITE);
  /* if open succesfully -> root != NULL 
    then write string "Hello world!" to it
  */
  if (root) {
    root.println(laura_angle);
    root.flush();
   /* close the file */
    root.close();
  } else {
    /* if the file open error, print an error */
    Serial.println("error opening test.txt");
  }
  
//  Serial.print("Angle:\t");
//  Serial.print(laura_angle,4);
//  Serial.println();

  // read raw gyro measurements from device
  BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);

  // display tab-separated  x/y/z values
//  Serial.print("g:\t");
//  Serial.print(gx);
//  Serial.print("\t");
//  Serial.print(gy);
//  Serial.print("\t");
//  Serial.print(gz);
//  Serial.println();
  
//  Serial.print("a:\t");
//  Serial.print(ax);
//  Serial.print("\t");
//  Serial.print(ay);
//  Serial.print("\t");
//  Serial.print(az);
//  Serial.println();


}

void printDirectory(File dir, int numTabs) {
  
  while(true) {
     File entry =  dir.openNextFile();
     if (! entry) {
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');   // we'll have a nice indentation
     }
     // Print the name
     Serial.print(entry.name());
     /* Recurse for directories, otherwise print the file size */
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       /* files have sizes, directories do not */
       Serial.print("\t\t");
       Serial.println(entry.size());
     }
     entry.close();
   }
}
