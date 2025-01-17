/*ESP32 Connections:
  PMS5003:   TX -> P4   RX -> P16
  BME280:    SCL -> P22   SDI  -> P21
  Display:    RST -> P2   CE -> P15   DC -> P5  DIN ->  P23  CLK -> P18      
*/

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
#include "PMS.h"
#include "Wire.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include<esp_task_wdt.h>

Adafruit_PCD8544 display = Adafruit_PCD8544(18,23,5,15,2);
Adafruit_BME280 bme; // I2C
PMS pms(Serial2);
PMS::DATA data;
uint32_t cnt = 1;

esp_task_wdt_config_t  wd_config;
uint32_t display_time = 0;


const unsigned char epd_bitmap_Untitled [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x63, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 
	0x60, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x00, 0xc7, 0x84, 0x23, 
	0x0f, 0xc7, 0x83, 0xc0, 0x01, 0xc0, 0x60, 0x00, 0x4c, 0xc6, 0x23, 0x08, 0xcc, 0xc6, 0x60, 0x07, 
	0x00, 0x60, 0x3f, 0xc0, 0x66, 0x22, 0x08, 0xc0, 0x60, 0x30, 0x1c, 0x07, 0xc0, 0x3f, 0xc0, 0x63, 
	0x26, 0x08, 0xc0, 0x20, 0x30, 0x70, 0x07, 0xc0, 0x00, 0x4f, 0xe1, 0xfc, 0x18, 0xc7, 0xe7, 0xf0, 
	0x38, 0x00, 0x60, 0x00, 0xc0, 0x23, 0x26, 0x18, 0xc0, 0x20, 0x30, 0x0e, 0x00, 0x30, 0x00, 0xc0, 
	0x66, 0x22, 0x10, 0xc0, 0x60, 0x30, 0x03, 0x80, 0x30, 0x61, 0x88, 0x44, 0x23, 0x30, 0xc8, 0x64, 
	0x60, 0x00, 0xc8, 0x60, 0x3f, 0x0f, 0xcc, 0x21, 0x7f, 0xef, 0xc7, 0xc0, 0x00, 0x0f, 0xc0, 0x00, 
	0x00, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00
};



void setup() {
  //enable watchdog
  wd_config.timeout_ms = 300000;
  esp_task_wdt_init(&wd_config);//enable panic so AirMonitor restarts
  esp_task_wdt_add(NULL);

  Serial.begin(115200);
  Serial2.begin(9600);
  pms.passiveMode();
  pms.wakeUp();

  //Nokia5110 display settings
  display.begin();
  display.setContrast(55);
  display.clearDisplay();
  display.display();

  // bme280
  bool status = bme.begin(0x76);  
  while (!status) {
    status = bme.begin(0x76);
    static int cntr=0;
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    if(cntr++>=10) break;
    delay(1);
  }
}


void display_values(uint16_t aqi, float temperature, uint16_t humidity){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.write("A: ");
    display.print(aqi);
    display.setCursor(0,17);
    display.write("T: ");
    display.print(temperature,1);
    display.setCursor(0,34);
    display.write("H: ");
    display.print(humidity);
    display.display();

}


float c_l,c_h,i_l,i_h;

int calculate_aqi(uint16_t concentration,float cl, float ch, float il, float ih){
  int AQI = ((ih - il)/(ch - cl)) * (concentration - cl) + il;
  return AQI;
}

int AQI_1_25(uint16_t concentration){
  if(concentration>=0 && concentration<=12){
    c_l=0; c_h = 12; i_l = 0; i_h = 50;
  }else if(concentration>=12.1	 && concentration <= 35.4){
    c_l=12.1; c_h = 35.4; i_l = 51; i_h = 100;
  }else if(concentration>=35.5	&& concentration <= 55.4){
    c_l=35.5; c_h = 55.4; i_l = 101; i_h = 150;
  }else if(concentration>=55.5 && concentration <= 150.4){
    c_l=55.5; c_h = 150.4; i_l = 151; i_h = 200;
  }else if(concentration>=150.5		 && concentration <= 250.4){
    c_l=150.5; c_h = 250.4; i_l = 201; i_h = 300;
  }else if(concentration>=250.5		 && concentration <= 350.4){
    c_l=250.5; c_h = 350.4; i_l = 301; i_h = 400;
  }else if(concentration>=350.5		&& concentration <= 500.4){
    c_l=350.5; c_h = 500.4; i_l = 401; i_h = 500;
  }else if(concentration>=500.5	 && concentration <= 999.9){
    c_l=500.5; c_h = 999.9; i_l = 501; i_h = 999;
  }else {
    c_l=0; c_h = 0; i_l = 0; i_h = 0;
  }
  return calculate_aqi(concentration, c_l, c_h, i_l, i_h);
}

int AQI_1_10(uint16_t concentration){
  if(concentration>=0 && concentration<=54){
    c_l=0; c_h = 54; i_l = 0; i_h = 50;
  }else if(concentration>=55  && concentration <= 154){
    c_l=55; c_h = 154; i_l = 51; i_h = 100;
  }else if(concentration>=155	&& concentration <= 254){
    c_l=35.5; c_h = 55.4; i_l = 101; i_h = 150;
  }else if(concentration>=255 && concentration <= 354){
    c_l=255; c_h = 354; i_l = 151; i_h = 200;
  }else if(concentration>=355		 && concentration <= 424){
    c_l=355; c_h = 424; i_l = 201; i_h = 300;
  }else if(concentration>=425	 && concentration <= 504){
    c_l=425; c_h = 504; i_l = 301; i_h = 400;
  }else if(concentration>=505	&& concentration <= 604){
    c_l=505; c_h = 604; i_l = 401; i_h = 500;
  }else if(concentration>=605	 && concentration <= 999){
    c_l=605; c_h = 999; i_l = 501; i_h = 999;
  }else {
    c_l=0; c_h = 0; i_l = 0; i_h = 0;
  }
  return calculate_aqi(concentration, c_l, c_h, i_l, i_h);

}

uint16_t get_aqi(float averages[]){
  uint16_t pm2_5_aqi;
  uint16_t pm10_aqi;
  pm2_5_aqi = AQI_1_25((uint16_t) averages[1]);
  pm10_aqi = AQI_1_10(averages[2]);
  if(pm2_5_aqi>pm10_aqi) pm10_aqi = pm2_5_aqi;
  return pm10_aqi;
}

float averages[10];
void loop() {
  esp_task_wdt_reset();
  Serial.println('.');
  pms.requestRead();
  if (pms.readUntil(data)) {
    averages[0] += data.PM_AE_UG_1_0;
    averages[1] += data.PM_AE_UG_2_5;
    averages[2] += data.PM_AE_UG_10_0;
  }else{
    Serial.println("PMS no data");
  }
  averages[3] += bme.readTemperature();
  averages[4] += bme.readHumidity();
  averages[5] += (bme.readPressure() / 100.0F);
  
  if (cnt++ % 10 == 0) { //print 10sec average values
    for(int i=0;i<10; i++) averages[i] = (float)averages[i]/10;
    averages[3]-=5; // approximate conpensation for bme280 sensor temperature readings

    Serial.print("PM 1.0 (ug/m3): ");
    Serial.println(averages[0]);

    Serial.print("PM 2.5 (ug/m3): ");
    Serial.println(averages[1]);

    Serial.print("PM 10.0 (ug/m3): ");
    Serial.println(averages[2]);

    Serial.print("Temperature = ");
    Serial.print(averages[3]);
    Serial.println(" *C");

    Serial.print("Humidity = ");
    Serial.print(averages[4]);
    Serial.println(" %");

    Serial.print("Pressure = ");
    Serial.print(averages[5]);
    Serial.println(" hPa");
    
    uint16_t aqi_index = get_aqi(averages);
    Serial.print("AQI = ");
    Serial.println(aqi_index);

    display_values(aqi_index, averages[3], averages[4]);
    memset(averages,0,sizeof(averages));
  }

  // if(display_time++ % 60 > 0){ //break displaying measurements
  //   // display.clearDisplay();
  //   // display.drawBitmap(0, 0, epd_bitmap_Untitled, 84, 42, 1);
  //   // display.display();
  //   delay(3000);
  // }else{  //display measurements
    
  //   display.clearDisplay();
  //   display.setTextSize(3);
  //   display.setCursor(0,5);
  //   display.write(3);
  //   display.display();
  //   delay(1000);

  // }

  delay(1000);
}






