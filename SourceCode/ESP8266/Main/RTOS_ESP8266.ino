#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#define Rx D6             //define 9 là Rx
#define Tx D5            //define 10 là Tx  
// Template ID, Device Name and Auth Token are provided by the Blynk.Cloud
// See the Device Info tab, or Template settings
#define BLYNK_TEMPLATE_ID "TMPLNUXrf_Lw"
#define BLYNK_DEVICE_NAME "RTOS"
#define BLYNK_AUTH_TOKEN "6eCA-6dQBVGoT_fCIc48mA4oIPRYHtSN"

// Comment this out to disable prints and save space

SoftwareSerial mySerial(Rx, Tx); //Khởi tạo cổng serial mềm


char auth[] = BLYNK_AUTH_TOKEN ; // Auth token

char ssid[] = "CoE-C1"; // Network SSID
char pass[] = "publicwifi"; // Network passowrd

//char ssid[] = "Truc Quynh"; // Network SSID
//char pass[] = "dungsithanhkhe"; // Network passowrd

BlynkTimer timer;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

int a;

WidgetLED led1(V2);
WidgetLED led2(V3);
WidgetLED led3(V4);
WidgetLED led4(V5);
WidgetLED led5(V6);

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  mySerial.begin(115200);
  while (!mySerial);
  Blynk.begin(auth, ssid, pass);
  pinMode(D0, OUTPUT);
  timer.setInterval(500L, buttonLedWidget);
}
void loop()
{
  Blynk.run();
  timer.run();
  Read_Uart();
}
void Read_Uart() {
  //DynamicJsonDocument doc(2048);
  StaticJsonDocument<40> doc;  
  DeserializationError error = deserializeJson(doc, mySerial);

  if (error) {
    //Serial.println("Invalid Json Object");
    return;
  }

  a = doc["Zone"];
  Serial.print("Zone: ");
  Serial.println(a);
}

void buttonLedWidget()
{
  if (a == 1) {
    led2.off();
    led3.off();
    led4.off();
    led5.off();
    led1.on();
  }
  else if (a == 2) {
    led1.off();
    led3.off();
    led4.off();
    led5.off();
    led2.on();
  }
  else if (a == 3) {
    led1.off();
    led2.off();
    led4.off();
    led5.off();
    led3.on();
  }
  else if (a == 4) {
    led1.off();
    led2.off();
    led3.off();
    led5.off();
    led4.on();
  }
  else if (a == 5) {
    led1.off();
    led2.off();
    led3.off();
    led4.off();
    led5.on();
  }
  else if ( a == 6) {
    led1.off();
    led2.off();
    led3.off();
    led4.off();
    led5.off();
  }

}

BLYNK_WRITE(V0)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.print("V0 Slider value is: ");
  Serial.println(pinValue);
  digitalWrite(D0, pinValue);
}
