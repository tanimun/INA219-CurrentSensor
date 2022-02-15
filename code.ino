#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <U8glib.h>
#include <Adafruit_INA219.h>
#include <SD.h>
#include <EEPROM.h>
//#include <LowPower.h>

#define sd_chipSelect 10
//#define btn_power 2
#define btn_record 4
#define btn_precision 5
#define btn_delay_millis 30
#define u8g_logo_width 128
#define u8g_logo_height 26

Adafruit_INA219 ina219;                                                               //SCL to Pin A5, SDA to Pin A4
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); // Fast I2C / TWI， D0(SCL/CLK) to Pin A5, D1(SDA/MOSI) to Pin A4, RES to RST/VCC, DC to GND, CS to GND
File myFile;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage_V = 0;
float power_mW = 0;
float mAh = 0;
float energy_mWh = 0;

unsigned long tick;
unsigned long lastread;
unsigned long previousMillis;

int recording_status = 0;
int sd_status = 0;
int precision_status = 0;
int power_status = 0;

int max_voltage = 0;
int max_current = 0;

const char file_name[] = "PM.csv";
const int interval = 200;

const char a1[] PROGMEM = "Not recording";
const char a2[] PROGMEM = "Recording...";
const char a3[] PROGMEM = "SD Card not avaliable";
const char *recording_status_text[] = {a1, a2, a3};

const char b1[] PROGMEM = "Limit 32V/2A";
const char b2[] PROGMEM = "Limit 32V/1A";
const char b3[] PROGMEM = "Limit 16V/400mA";
static const char *const precision_text[] = {b1, b2, b3};

static unsigned char u8g_logo_bits[] U8G_PROGMEM = {
    0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x1f, 0x00, 0x00, 0x00, 0x00,
    0x00, 0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x1f, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xf0, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x87, 0x0f, 0x00, 0x30,
    0x00, 0x00, 0x00, 0xf0, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x87, 0x0f, 0x00,
    0x3c, 0x00, 0x00, 0x00, 0xf0, 0x78, 0xf8, 0xf0, 0xf9, 0x79, 0x7c, 0x70, 0xc7, 0xcf, 0x0f,
    0x1e, 0xff, 0xe1, 0xc1, 0x19, 0xf0, 0x78, 0xfe, 0xf9, 0xfb, 0x79, 0xfe, 0xfc, 0xcf, 0xcf,
    0x8f, 0x7f, 0xff, 0xf9, 0xf7, 0x3f, 0xf0, 0x7c, 0xcf, 0xe3, 0xf3, 0x19, 0xe7, 0xfd, 0xcf,
    0xff, 0x8f, 0x73, 0x3c, 0x3c, 0xe7, 0x3f, 0xf0, 0x3f, 0xcf, 0xe7, 0xfb, 0x9d, 0xff, 0x79,
    0xc7, 0xff, 0xcf, 0xff, 0x3c, 0xfc, 0xef, 0x1b, 0xf0, 0x0f, 0xcf, 0xc7, 0xff, 0x8f, 0xff,
    0x79, 0xc0, 0x7e, 0xcf, 0xff, 0x3c, 0xfc, 0xe7, 0x01, 0xf0, 0x00, 0x8f, 0xc7, 0xff, 0x8f,
    0x07, 0x78, 0xc0, 0x3c, 0xcf, 0x03, 0x3c, 0x3c, 0xe0, 0x01, 0xf0, 0x00, 0x8f, 0x87, 0xcf,
    0x87, 0x07, 0x78, 0xc0, 0x3c, 0xcf, 0x03, 0x3c, 0x3c, 0xe0, 0x03, 0xf0, 0x00, 0xdf, 0x03,
    0x8f, 0x07, 0x8f, 0x7c, 0xe0, 0x1c, 0x8f, 0x47, 0x3c, 0x7c, 0xe6, 0x03, 0xf8, 0x01, 0xfe,
    0x01, 0x87, 0x03, 0xfe, 0xfc, 0xf0, 0x99, 0x9f, 0x7f, 0xfc, 0xf9, 0xf7, 0x07, 0xf8, 0x01,
    0x7c, 0x00, 0x06, 0x01, 0x7c, 0xfc, 0xe0, 0x89, 0x1f, 0x1e, 0xf8, 0xe0, 0xf1, 0x07, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0xe0, 0x61, 0x7c, 0x36,
    0xf3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x30, 0x62, 0xcc,
    0x76, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x30, 0xf6,
    0x8c, 0xf6, 0xcb, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x30,
    0xf2, 0xcd, 0x96, 0x1b, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0,
    0xef, 0x19, 0x7f, 0x16, 0xf3, 0x49, 0x02, 0x00, 0x00, 0x00, 0x00};

void setup(void)
{
  //pinMode(btn_power, INPUT_PULLUP);
  //Serial.begin(9600);
  pinMode(btn_record, INPUT_PULLUP);
  pinMode(btn_precision, INPUT_PULLUP);

  u8g.begin();
  u8g.firstPage();
  do
  {
    u8g.drawXBMP(0, 19, u8g_logo_width, u8g_logo_height, u8g_logo_bits);

  } while (u8g.nextPage());
  ina219.begin();
  precision_status = EEPROM.read(0);
  set_precision(precision_status); //设置INA219采样精度

  //检查SD卡是否可用
  SD.begin(sd_chipSelect);
  if (SD.open(file_name, FILE_WRITE))
  {
    sd_status = 1;
    //Serial.print("SD Card avaliable");
    myFile.close();
  }
  SD.end();
  //attachInterrupt(digitalPinToInterrupt(btn_power), wakeUp, LOW);
  delay(1000);
  previousMillis, lastread = millis();
}

void loop(void)
{
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  btn_record_click();    //切换记录状态
  btn_precision_click(); //切换INA219精度
  unsigned long currentMillis = millis();

  if ((currentMillis - previousMillis) >= interval)
  {
    previousMillis = currentMillis;
    getinfo();
    draw_info();
    record_info();
  }
}

// void wakeUp()
// {
// }

void getinfo()
{
  unsigned long newtime;
  shuntvoltage = ina219.getShuntVoltage_mV(); //分流电阻的电压，最高320mV
  busvoltage = ina219.getBusVoltage_V();      //负载电压
  current_mA = ina219.getCurrent_mA();        //回路电流
  //current_mA=400.0;
  power_mW = ina219.getPower_mW();
  loadvoltage_V = busvoltage + (shuntvoltage / 1000.0);
  newtime = millis();
  tick = newtime - lastread;
  mAh += current_mA * tick / 3600000.0;
  energy_mWh += power_mW * tick / 3600000.0;
  lastread = newtime;
}

void draw_info()
{
  u8g.firstPage();
  do
  {
    u8g.setFont(u8g_font_fixed_v0); // 设置数据字体
    int n = 13;

    if (loadvoltage_V >= max_voltage)
    {
      u8g.drawStr(2, n, "OverLoad");
    }
    else
    {
     u8g.drawStr(2, n, convert(loadvoltage_V, 3, 2, "V"));
    }
    
    if (current_mA >= max_current)
    {
      u8g.drawStr(60, n, "OverLoad");
    }
    else
    {
      if (current_mA < 1000)
        u8g.drawStr(60, n, convert(current_mA, 3, 2, "mA"));
      else
        u8g.drawStr(60, n, convert(current_mA / 1000, 3, 2, "A"));
    }

    n += 13;
    if (power_mW < 1000)
      u8g.drawStr(2, n, convert(power_mW, 3, 2, "mW"));
    else
      u8g.drawStr(2, n, convert(power_mW / 1000, 3, 2, "W"));
    if (mAh < 1000)
      u8g.drawStr(60, n, convert(mAh, 3, 2, "mAh"));
    else
      u8g.drawStr(60, n, convert(mAh / 1000, 3, 2, "Ah"));

    n += 13;
    if (mAh < 1000)
      u8g.drawStr(2, n, convert(energy_mWh, 3, 2, "mWh"));
    else
      u8g.drawStr(2, n, convert(energy_mWh / 1000, 3, 2, "Wh"));

    u8g.setFont(u8g_font_profont10r); // 设置状态字体
    n += 14;
    u8g.drawStr(2, 54, (class __FlashStringHelper *)precision_text[precision_status]);
    n += 14;
    if (!sd_status)
    {
      u8g.drawStr(2, 64, (class __FlashStringHelper *)recording_status_text[2]);
    }
    else
    {
      u8g.drawStr(2, 64, (class __FlashStringHelper *)recording_status_text[recording_status]);
    }

  } while (u8g.nextPage());
}

void record_info()
{
  //  Serial.print("recording_status=");
  //  Serial.println(recording_status);
  if (!recording_status)
  {
    return;
  }

  myFile = SD.open(file_name, FILE_WRITE);

  if (myFile)
  {
    // Serial.print("Writing to ");
    // Serial.print(file_name);
    // Serial.print("...");
    myFile.println(collect_data());
    // close the file:
    myFile.close();
    // Serial.println("done.");
  }
  else
  {
    // if the file didn't open, print an error:
    // Serial.print("error opening ");
    // Serial.println(file_name);
    sd_status = 0;        //写入过程如果出错，则将SD卡状态切换为不可用
    recording_status = 0; //写入过程如果出错，则把记录状态切换为停止记录
    // Serial.println("SD Card not avaliable");
    // Serial.print("recording_status=");
    // Serial.println(recording_status);
    SD.end(); //如果无法写入文件头，则调用SD.end(),在下一次访问SD卡前重新调用SD.begin()，否则会一直无法访问SD卡
  }
}

void set_precision(int precision_status)
{
  switch (precision_status)
  {
  case 0:
    ina219.setCalibration_32V_2A();
    max_voltage = 32;
    max_current = 2000;
    break;
  case 1:
    ina219.setCalibration_32V_1A();
    max_voltage = 32;
    max_current = 1000;
    break;
  case 2:
    ina219.setCalibration_16V_400mA();
    max_voltage = 16;
    max_current = 400;
    break;
  }
}

void btn_precision_click()
{
  int buttonState = digitalRead(btn_precision);
  if (buttonState == LOW)
  {
    delay(btn_delay_millis);
    if (buttonState == LOW)
    {
      precision_status += 1;
      if (precision_status > 2)
      {
        precision_status = 0;
      }
      set_precision(precision_status);
      EEPROM.write(0, precision_status);
      // Serial.print("precision:");
      // Serial.println(precision_text[precision_status]);
      while (digitalRead(btn_precision) == LOW)
        ;
    }
  }
}

void btn_record_click()
{
  int buttonState = digitalRead(btn_record);
  if (buttonState == LOW)
  {
    delay(btn_delay_millis);
    if (buttonState == LOW)
    {
      recording_status = !recording_status;
      if (recording_status)
      {
        u8g.firstPage();
        do
        {
          u8g.setFont(u8g_font_fixed_v0); // 设置数据字体
          u8g.drawStr(2, 13, F("Reading SD Card..."));

        } while (u8g.nextPage());
        SD.begin(sd_chipSelect); //如果写入状态为正在记录，则重新调用SD.begin()，因为每次出现写入错误或无SD卡都会调用SD.end()
        myFile = SD.open(file_name, FILE_WRITE);
        if (myFile)
        {
          sd_status = 1; //如果正常打开文件，则把记录状态切换为正在记录
          // Serial.print("Writing to loadvoltage_V,current_mA,power_mW,energy_mWh，interval_ms");
          myFile.println(F("loadvoltage_V,current_mA,power_mW,mAh,energy_mWh,interval_ms"));
          // close the file:
          myFile.close();
          // Serial.println("done.");
        }
        else
        {
          // if the file didn't open, print an error:
          // Serial.print("error opening ");
          // Serial.println(file_name);
          sd_status = 0;        //如果无法写入文件头，则将SD卡状态切换为不可用
          recording_status = 0; //如果无法写入文件头，则把记录状态切换为停止记录
          // Serial.println("SD Card not avaliable");
          // Serial.print("recording_status=");
          // Serial.println(recording_status);
          SD.end(); //如果无法写入文件头，则调用SD.end(),在下一次访问SD卡前重新调用SD.begin()，否则会一直无法访问SD卡
        }
      }
      while (digitalRead(btn_record) == LOW)
        ;
      // Serial.print("recording_status=");
      // Serial.println(recording_status);
    }
  }
}

const char *collect_data()
{
  static char str[7];
  static char buf[39];
  dtostrf(loadvoltage_V, 3, 2, str);
  strcpy(buf, str);
  strcat(buf, ",");

  dtostrf(current_mA, 3, 2, str);
  strcat(buf, str);
  strcat(buf, ",");

  dtostrf(power_mW, 3, 2, str);
  strcat(buf, str);
  strcat(buf, ",");

  dtostrf(mAh, 3, 2, str);
  strcat(buf, str);
  strcat(buf, ",");

  dtostrf(energy_mWh, 3, 2, str);
  strcat(buf, str);
  strcat(buf, ",");

  dtostrf(tick, 3, 0, str);
  strcat(buf, str);
  return buf;
}

const char *convert(float val, int minStringWidthIncDecimalPoint, int numVarsAfterDecimal, char a[])
{
  static char buf[10];
  static char str[7];
  dtostrf(val, minStringWidthIncDecimalPoint, numVarsAfterDecimal, str);
  strcpy(buf, str);
  strcat(buf, a);
  return buf;
}
