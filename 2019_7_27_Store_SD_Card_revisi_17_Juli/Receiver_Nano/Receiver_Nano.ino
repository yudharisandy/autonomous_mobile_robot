//receiver
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
SoftwareSerial serial(10, 9);
String data;
const int chipSelect = 4;
File dataFile;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  serial.begin(38400);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");

  // Open up the file we're going to log to!
  dataFile = SD.open("data2.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) {
    //Serial.println(serial.read());
    delay(10);
    char c = Serial.read();
    data += c;

  } if (data.length() > 0) {
    dataFile.println(data);
    Serial.println(data);
    data = "";
  }
  dataFile.flush();
  delay(500);
}
