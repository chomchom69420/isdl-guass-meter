#include <Arduino.h>
#include <ADS1X15.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ESPmDNS.h>
#include "ESPTelnet.h"
#include <WebSerial.h>

#define WINDOW_SIZE 50

WiFiServer TelnetServer(23);
WiFiClient Telnet;

void handleTelnet() {
  if (TelnetServer.hasClient()) {
    //client is connected
    if (!Telnet || !Telnet.connected()) {
      if (Telnet) Telnet.stop();          //client disconnected
      Telnet = TelnetServer.available();  //ready for new client
    }
    else {
      TelnetServer.available().stop(); //have client, block new clients
    }
  }
}

const char* ssid = "Soham_TPLINK";
const char* password = "sohamc621";
uint8_t Wifi_tries = 0;
bool Wifi_flag = 0;

AsyncWebServer server(80);

#define HOLD_PIN  13
#define NULL_PIN  12

ADS1115 ADS(0x4A);

//ADS1115 configurations
uint8_t sample_rate = 7;    //860 SPS (fastest)
uint8_t adc_mode    = 0;    //Continuous mode 
uint8_t adc_pin     = 0;    //Pin 0 of ADC

const int lcdColumns = 16;
const int lcdRows = 2;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

//ADC value
static int16_t adc_val;
static float sens_volt;
const float gain=1.64;

//Gauss value
float gauss_val=0;
float avg_gauss_val=0;
volatile float offset=0;
volatile float hold_val=0;
volatile bool hold_flag=false; //0 for not hold, 1 for hold

//Moving average
float data[WINDOW_SIZE] = {0};

//Debounce variables
unsigned long debounceDelay = 25; 
unsigned long null_lastDebounceTime = 0; 
int null_buttonState;            
int null_lastButtonState = HIGH; 
unsigned long hold_lastDebounceTime = 0; 
int hold_buttonState;            
int hold_lastButtonState = HIGH;  

unsigned long lcdDelay = 95;
unsigned long last_lcdTime = 0;

float moving_average(float *data, int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

void readADCTask(void* parameters) {
  adc_val = ADS.getValue();
  sens_volt = ADS.toVoltage(adc_val);
}

void printTask(void* parameters) {
  WebSerial.print("Sensor voltage: ");
  WebSerial.print(sens_volt, 5);
  WebSerial.println();
}

void processingTask(void *parameters) {
  gauss_val = ((sens_volt-2.5)*1000)/(1.4*gain);

  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    data[i] = data[i + 1];
  }
  data[WINDOW_SIZE - 1] = gauss_val;
  avg_gauss_val = moving_average(data, WINDOW_SIZE);
}

void LCDTask(void *parameters) {
  lcd.setCursor(0,0);
  char s[50];
  float print_val=0;
  if (hold_flag)
    print_val = hold_val;
  else
    print_val = avg_gauss_val - offset;
  sprintf(s, "Gauss: %.2f", print_val);
  lcd.print(s);
}

void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.begin(115200);
  delay(100);
  Serial.println("");

  while (WiFi.status() != WL_CONNECTED) {
    if (Wifi_tries < 10) {
      delay(500);
      Serial.print(".");
      Wifi_tries++;
    }
    else {
      break;
    }
  }

  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    TelnetServer.begin();
    TelnetServer.setNoDelay(true);

    // Initialize mDNS
    if (!MDNS.begin("esp32")) {   // Set the hostname to "esp32.local"
      Serial.println("Error setting up MDNS responder!");
      while(1) {
        delay(1000);
      }
    }
    Serial.println("mDNS responder started");

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Gauss Meter v1.1 by ISDL Lab Group 1: Soham "
                                      "Chakraborty, Mukut Debnath, Panthadip Maji");
    });

    AsyncElegantOTA.begin(&server);    
    WebSerial.begin(&server);
    WebSerial.msgCallback(recvMsg);
    server.begin();
    Serial.println("HTTP server started");
    delay(2000);
  } 
  if (WiFi.status() == WL_CONNECTED)  
    WebSerial.println("WL_CONNECTED");

  pinMode(NULL_PIN, INPUT_PULLUP);
  pinMode(HOLD_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight(); 

  Wire.begin();

  while (!ADS.isConnected()) {
    if (WiFi.status() == WL_CONNECTED)
      WebSerial.println("ADS1115 is not connected. Trying again...");
    vTaskDelay(1000 / portMAX_DELAY);
  }
  if (WiFi.status() == WL_CONNECTED)
    WebSerial.println("ADS1115 connected!");

  if (WiFi.status() == WL_CONNECTED)
    WebSerial.println("Setting up ADS1115...");
  ADS.begin();
  ADS.setGain(0);
  ADS.setDataRate(sample_rate);
  //Set continuous mode 
  ADS.setMode(adc_mode);
  //Start conversion
  ADS.requestADC(adc_pin);

  delay(10);
  if (WiFi.status() == WL_CONNECTED)
    WebSerial.println("Finished configuring ADS1115.");

  last_lcdTime = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED)
    handleTelnet();
  readADCTask((void*)NULL);
  processingTask((void*)NULL);

  int null_reading = digitalRead(NULL_PIN);
  int hold_reading = digitalRead(HOLD_PIN);

  if (null_reading != null_lastButtonState) {
    null_lastDebounceTime = millis();
  }

  if (hold_reading != hold_lastButtonState) {
    hold_lastDebounceTime = millis();
  }

  if ((millis() - null_lastDebounceTime) > debounceDelay) {

    if (null_reading != null_buttonState) {
      null_buttonState = null_reading;
      if (null_buttonState == LOW) {
        offset = avg_gauss_val;
        if (WiFi.status() == WL_CONNECTED) {
          WebSerial.println("Null button pressed.");
          WebSerial.printf("Offset: %.5f\n", offset);
        }
        lcd.clear();
      }
    }
  }

  if ((millis() - hold_lastDebounceTime) > debounceDelay) {
    if (hold_reading != hold_buttonState) {
      hold_buttonState = hold_reading;
      if (hold_buttonState == LOW) {
        hold_flag = !hold_flag;
        if(hold_flag) 
          hold_val = avg_gauss_val - offset;
          if (WiFi.status() == WL_CONNECTED) 
            WebSerial.println("Hold button pressed.");
      }
    }
  }

  if (millis() - last_lcdTime > lcdDelay) {
    LCDTask((void*)NULL);
    last_lcdTime = millis();
  }
  null_lastButtonState = null_reading;
  hold_lastButtonState = hold_reading;
  delay(10);
}

