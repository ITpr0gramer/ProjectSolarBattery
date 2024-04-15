/*
 MODIFIED BY MD JAHID HASSAN
 ME,CUET
 BANGLADESH
*/
#include <Wire.h>
#include <Servo.h>
// Power Питание
#define BH1750_POWER_DOWN 0x00  // Нет активного состояния
#define BH1750_POWER_ON 0x01    // Ожидание команды измерения
#define BH1750_RESET 0x07       // Сброс значения регистра данных - не принимается в режиме POWER_DOWN

// Режим измерения
#define CONTINUOUS_HIGH_RES_MODE 0x10    // Measurement at 1 lux resolution. Measurement time is approx 120ms
#define CONTINUOUS_HIGH_RES_MODE_2 0x11  // Measurement at 0.5 lux resolution. Measurement time is approx 120ms
#define CONTINUOUS_LOW_RES_MODE 0x13     // Measurement at 4 lux resolution. Measurement time is approx 16ms
#define ONE_TIME_HIGH_RES_MODE 0x20      // Measurement at 1 lux resolution. Measurement time is approx 120ms
#define ONE_TIME_HIGH_RES_MODE_2 0x21    // Measurement at 0.5 lux resolution. Measurement time is approx 120ms
#define ONE_TIME_LOW_RES_MODE 0x23       // Measurement at 4 lux resolution. Measurement time is approx 16ms

// I2C Address
#define BH1750_1_ADDRESS 0x23  // датчик 1 подключен на GND
#define BH1750_2_ADDRESS 0x5C  // датчик 2 подключен на VCC


// Определение переменной
int16_t s_en = 4;
int16_t A = 5;
int16_t B = 6;
int16_t C = 7;

int16_t RawData;
int16_t SensorValue[4];

//------------сервопривод int

// 180 градусов максимально
Servo horizontal;  
int servoh = 90;   // 90;     // горизонтальный сервопривод

int servohLimitHigh = 180;
int servohLimitLow = 65;

// 180 градусов максимальн
Servo vertical;  
int servov = 90;  //   90;     // вертикальный сервопривод

int servovLimitHigh = 120;
int servovLimitLow = 15;






void setup() {
  Wire.begin();
  Serial.begin(115200);  // скорость передачи

  pinMode(s_en, OUTPUT);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
  pinMode(C, OUTPUT);

  digitalWrite(s_en, HIGH);
  digitalWrite(A, HIGH);
  digitalWrite(B, HIGH);
  digitalWrite(C, HIGH);

  horizontal.attach(10);
  vertical.attach(9);
  horizontal.write(180);
  vertical.write(45);
  delay(3000);
}

void loop() {

  digitalWrite(s_en, LOW);  // включение переключателя Enable

  for (int i = 0; i < 4; i++) {
    readMux(i);

    init_BH1750(BH1750_1_ADDRESS, CONTINUOUS_HIGH_RES_MODE);
    delay(120);
    RawData_BH1750(BH1750_1_ADDRESS);
    SensorValue[i] = RawData / 1.2;
    // delay(20);
  }

  Serial.print("Sensor_1 = ");
  Serial.print(SensorValue[0]);
  Serial.print(" | Sensor_2 = ");
  Serial.print(SensorValue[1]);
  Serial.print(" | Sensor_3 = ");
  Serial.print(SensorValue[2]);
  Serial.print(" | Sensor_4 = ");
  Serial.println(SensorValue[3]);

  int lt = SensorValue[2];  // вверху слева
  int rt = SensorValue[3];  // вверху справа
  int ld = SensorValue[1];  // внизу слева
  int rd = SensorValue[0]; // внизу справа

  int dtime = 10;
  int tol = 50;

  int avt = (lt + rt) / 2;  // среднее значение вверху
  int avd = (ld + rd) / 2;  // среднее значение внизу
  int avl = (lt + ld) / 2;  // среднее значение слева
  int avr = (rt + rd) / 2;  // среднее значение справа

  int dvert = avt - avd;   // проверить разницу между up and down
  int dhoriz = avl - avr;  // проверить разницу между left and rigt


  Serial.print(avt);
  Serial.print(" ");
  Serial.print(avd);
  Serial.print(" ");
  Serial.print(avl);
  Serial.print(" ");
  Serial.print(avr);
  Serial.print("   ");
  Serial.print(dtime);
  Serial.print("   ");
  Serial.print(tol);
  Serial.println(" ");


  if (-1 * tol > dvert || dvert > tol)  // проверьте, есть ли разница в допуске, иначе измените вертикальный угол
    if (avt > avd) {
      servov = ++servov;
      if (servov > servovLimitHigh) {
        servov = servovLimitHigh;
      }
    } else if (avt < avd) {
      servov = --servov;
      if (servov < servovLimitLow) {
        servov = servovLimitLow;
      }
    }
    vertical.write(servov);
  }

  if (-1 * tol > dhoriz || dhoriz > tol)  // проверьте, есть ли разница в допуске, иначе измените вертикальный угол
  {
    if (avl > avr) {
      servoh = --servoh;
      if (servoh < servohLimitLow) {
        servoh = servohLimitLow;
      }
    } else if (avl < avr) {
      servoh = ++servoh;
      if (servoh > servohLimitHigh) {
        servoh = servohLimitHigh;
      }
    } else if (avl = avr) {
      // ничего
    }
    horizontal.write(servoh);
  }
   delay(dtime);


void init_BH1750(int ADDRESS, int MODE) {
  //BH1750 инициализация и сброс
  Wire.beginTransmission(ADDRESS);
  Wire.write(MODE);  // PWR_MGMT_1 регистр
  Wire.endTransmission(true);
}

void RawData_BH1750(int ADDRESS) {
  Wire.beginTransmission(ADDRESS);
  Wire.requestFrom(ADDRESS, 2, true);        // запрашивать 2 регистра
  RawData = Wire.read() << 8 | Wire.read();  // считывать необработанные данные BH1750
  Wire.endTransmission(true);
}

int readMux(int channel) {
  int controlPin[] = { A, B, C };
  int muxChannel[8][3] = {
    { 0, 0, 0 },  //channel 0
    { 1, 0, 0 },  //channel 1
    { 0, 1, 0 },  //channel 2
    { 1, 1, 0 },  //channel 3
    // {0,0,1}, //channel 4
    // {1,0,1}, //channel 5
    // {0,1,1}, //channel 6
    // {1,1,1}, //channel 7  //loop through the 3 Signals
  };
  for (int i = 0; i < 3; i++) {  // Connecting MUX Channel
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
}
