#include <Arduino.h>
#include <ADS1X15.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define HOLD_PIN  34
#define NULL_PIN  35

// initialize ADS1115 on I2C bus 1 with default address 0x48
ADS1115 ADS(0x48);

//ADS1115 configurations
uint8_t sample_rate = 7;    //860 SPS (fastest)
uint8_t adc_mode    = 0;    //Continuous mode 
uint8_t adc_pin     = 0;    //Pin 0 of ADC

//LCD configuration
const int lcdColumns = 16;
const int lcdRows = 2;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

//ADC value
static int16_t adc_val;
static float sens_volt;
const float gain=1.64;

//Gauss value
static float gauss_val;
volatile float offset=0;
volatile float hold_val=0;
volatile bool hold_flag=false; //0 for not hold, 1 for hold

void IRAM_ATTR holdISR() {
  hold_flag = !hold_flag;
  if(hold_flag) hold_val = gauss_val - offset;
}

void IRAM_ATTR nullISR() {
  offset = gauss_val;
}

void readADCTask(void* parameters) {
  adc_val = ADS.getValue();
  sens_volt = ADS.toVoltage(adc_val);

}

void printTask(void* parameters) {
  Serial.print("Sensor voltage: ");
  Serial.print(sens_volt, 5);
  Serial.println();
}

void processingTask(void *parameters) {
  gauss_val = ((sens_volt-2.5)*1000)/(1.4*gain);
}

void LCDTask(void *parameters) {
  lcd.setCursor(0,0);
  char s[50];
  float print_val=0;
  if (hold_flag)
    print_val = hold_val;
  else
    print_val = gauss_val - offset;
  sprintf(s, "Gauss: %.3f", print_val);
  lcd.print(s);
}

void setup() {
  Serial.begin(115200);

  Wire.begin();

  while (!ADS.isConnected()) {
    // error ADS1115 not connected
    Serial.println("ADS1115 is not cSonnected");
    vTaskDelay(1000 / portMAX_DELAY);
  }

  ADS.begin();
  ADS.setGain(0);
  ADS.setDataRate(sample_rate);
  //Set continuous mode 
  ADS.setMode(adc_mode);
  //Start conversion
  ADS.requestADC(adc_pin);

  //Init lcd 
  lcd.init();
  lcd.backlight(); 

  //Attach interrupts
  attachInterrupt(HOLD_PIN, holdISR, RISING);
  attachInterrupt(NULL_PIN, nullISR, RISING);
}

void loop() {
  readADCTask((void*)NULL);
  processingTask((void*)NULL);
  printTask((void*)NULL);
  LCDTask((void*)NULL);
  delay(500);
}

