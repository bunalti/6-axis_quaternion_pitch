  #include <BMI160Gen.h>


#include <ESPNtpClient.h>
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#ifndef WIFI_CONFIG_H
#define YOUR_WIFI_SSID "SUPERONLINE_WiFi_7815"
#define YOUR_WIFI_PASSWD "NP9HFHXHJH7M"
#endif // !WIFI_CONFIG_H

#include <mySD.h>



File root;

const int i2c_addr = 0x69;
int16_t gx, gy, gz, ax, ay, az;         // raw values

unsigned long Start = 0;
float delta,wx,wy,wz;



// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;




void taskNTP( void * parameter ) {

  while (true) {
    formattedDate = NTP.getTimeDateStringUs();
    int splitT = formattedDate.indexOf(" ");
    timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-4);
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void setup() {


  WiFi.begin (YOUR_WIFI_SSID, YOUR_WIFI_PASSWD);
  NTP.setTimeZone (TZ_Europe_Istanbul);
  NTP.begin ();

  pinMode(2,OUTPUT);
  
  Serial.begin(115200); // initialize Serial communication
  while (!Serial);    // wait for the serial port to open
  


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


  BMI160.setGyroRange(2000);
  Serial.print("Gyro Range: \t");
  Serial.print(BMI160.getGyroRange());
  Serial.println();
  Serial.print("Acc Range: \t");
  Serial.print(BMI160.getAccelerometerRange());
  Serial.println();


  Serial.print("Initializing SD card...");
  /* initialize SD library with Soft SPI pins, if using Hard SPI replace with this SD.begin()*/
  // SCK, MISO, MOSI ,SS
  if (!SD.begin(26, 14, 13, 27)) {
    Serial.println("initialization failed!");
    while(1);
  }
  Serial.println("initialization done.");


  xTaskCreatePinnedToCore(taskNTP, "NTP", 5000, __null, 1, __null, 0);

 
  digitalWrite(2,HIGH);
  
}

void loop() {
  

  formattedDate = NTP.getTimeDateStringUs();
  int splitT = formattedDate.indexOf(" ");
  timeStamp = formattedDate.substring(splitT+1, formattedDate.length()-4);


  // Radians per sec
  
  wx = 0.00106422543*gx;
  wy = 0.00106422543*gy;
  wz = 0.00106422543*gz;

  float r_wz = wz*9.5492965964254;
  
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
    root.print(timeStamp);
    root.print(",\t");
    root.println(r_wz);
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
//  Serial.print(wx);
//  Serial.print("\t");
//  Serial.print(wy);
//  Serial.print("\t");
//  Serial.print(r_wz);
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
