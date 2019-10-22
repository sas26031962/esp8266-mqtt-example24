// Встроенный светодиод подлкючен к выводу D4 контроллера
// SSL port
// Подключён гироскоп MPU6050 по шине I2C по адресу 68


#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <Wire.h>
#include "Kalman.h"
 
//
// Константы
//

#define BUILT_IN_LED D4
#define DATA_REPEATE 300

const char *ssid1 = "WiFi-DOM.RU-3915"; // Имя WiFi точки доступа
const char *pass1 = "FzXbgYwPhU"; // Пароль от WiFi точки доступа

const char *ssid2 = "sas26031962"; // Имя WiFi точки доступа
const char *pass2 = "11111111"; // Пароль от WiFi точки доступа

/*
//Andy26031962 
const char *mqtt_server = "soldier.cloudmqtt.com"; // Имя сервера MQTT
const int mqtt_port = 15103; // Порт для подключения к серверу MQTT
//const int mqtt_port = 25103; // SSL порт для подключения к серверу MQTT
const char *mqtt_user = "xebuduol"; // Логи от сервер
const char *mqtt_pass = "C7uo-MebBHo1"; // Пароль от сервера

*/

//Andy 20190919
const char *mqtt_server = "soldier.cloudmqtt.com"; // Имя сервера MQTT
const int mqtt_port = 13381; // Порт для подключения к серверу MQTT
//const int mqtt_port = 23381; // SSL порт для подключения к серверу MQTT
const char *mqtt_user = "xvlfkffd"; // Логи от сервер
const char *mqtt_pass = "TmV4WHEGgXSk"; // Пароль от сервера

String sTopic_1 = "wemosd1r1_ledX";
String sTopic_2 = "wemosd1r1_ledY";

String sTopic_3 = "wemosd1r1_count";

String sTopic_4 = "wemosd1r1_lm75_temperature";

String sTopic_5 = "wemosd1r1_mpu6050_x";
String sTopic_6 = "wemosd1r1_mpu6050_y";



String sConnection = "Andy20190919";

#define BUFFER_SIZE 100

//
// Глобальные переменные
//
//---Гироскоп MPU6050---

Kalman kalmanX;
Kalman kalmanY;

uint8_t IMUAddress = 0x68;
 
/* IMU Data */
int16_t accX;
int16_t accY;
int16_t accZ;
int16_t tempRaw;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
 
double accXangle; // Angle calculate using the accelerometer
double accYangle;
double temp;
double gyroXangle = 180; // Angle calculate using the gyro
double gyroYangle = 180;
double compAngleX = 180; // Calculate the angle using a Kalman filter
double compAngleY = 180;
double kalAngleX; // Calculate the angle using a Kalman filter
double kalAngleY;
 
uint32_t timer;

//-----------------------
bool LedState = false;
int tm = DATA_REPEATE;

int Counter = 0;

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

//
// Функция получения данных от сервера
//
void callback(const MQTT::Publish& pub)
{
  Serial.print(pub.topic()); // выводим в сериал порт название топика
  Serial.print(" => ");
  Serial.print(pub.payload_string()); // выводим в сериал порт значение полученных данных

  String payload = pub.payload_string();

  if(String(pub.topic()) == sTopic_1)  // проверяем из нужного ли нам топика пришли данные
  {
    int stled = payload.toInt();      // преобразуем полученные данные в тип integer
    Serial.print(">>" + String(stled,DEC)+ "\n");
    digitalWrite(LED_BUILTIN,stled); // включаем или выключаем светодиод в зависимоти от полученных значений данных
    LedState = (stled > 0);
  }

  if(String(pub.topic()) == sTopic_2)  // проверяем из нужного ли нам топика пришли данные
  {
    int stled = payload.toInt();      // преобразуем полученные данные в тип integer
    Serial.print(">>" + String(stled,DEC)+ "\n");
    digitalWrite(LED_BUILTIN,stled); // включаем или выключаем светодиод в зависимоти от полученных значений данных
    LedState = (stled > 0);
  }
}//End of void callback(const MQTT::Publish& pub)

//
// Программа начальной установки
//
void setup() 
{
  
  Wire.begin();
  Serial.begin(115200);
    
 
  i2cWrite(0x6B,0x00); // Disable sleep mode      
  kalmanX.setAngle(180); // Set starting angle
  kalmanY.setAngle(180);
  timer = micros();

  delay(10);
  Serial.println();
  Serial.println();
  
  pinMode(LED_BUILTIN, OUTPUT);

}//End of void setup()

//
// Главный цикл программы
//
void loop() 
{
  // подключаемся к wi-fi
  if (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print("Connecting to ");
    Serial.print(ssid2);
    Serial.println("...");
    WiFi.begin(ssid2, pass2);

    if (WiFi.waitForConnectResult() == WL_CONNECTED)
    {
      Serial.println("WiFi connected to sas26031962");
    }
    else
    {
      return;
    }
  }

  // подключаемся к MQTT серверу
  if (WiFi.status() == WL_CONNECTED) 
  {
    if (!client.connected()) 
    {
      Serial.println("Connecting to MQTT server");
      if (client.connect(MQTT::Connect(sConnection).set_auth(mqtt_user, mqtt_pass))) 
      {
        Serial.println("Connected to MQTT server");
        client.set_callback(callback);
        client.subscribe(sTopic_1); // подписываемся по топик 1
        client.subscribe(sTopic_2); // подписываемся по топик 2
      } 
      else 
      {
        Serial.println("Could not connect to MQTT server");
      }
    }

    if (client.connected())
    {
      client.loop();
      DataSend();
    }

  }//End of if (WiFi.status() == WL_CONNECTED)

  else
  {
    Serial.print(".");
  }

}//End of void loop()

//
// Функция публикации данных
//
void DataSend()
{
  if (tm==0)
  {
    //... здесь помещается блок получения данных от гироскопа
    uint8_t* data = i2cRead(0x3B,14);
    accX = ((data[0] << 8) | data[1]);
    accY = ((data[2] << 8) | data[3]);
    accZ = ((data[4] << 8) | data[5]);
    tempRaw = ((data[6] << 8) | data[7]);
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);
    gyroZ = ((data[12] << 8) | data[13]);
 
    /* Calculate the angls based on the different sensors and algorithm */
    accYangle = (atan2(accX,accZ)+PI)*RAD_TO_DEG;
    accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;  
 
    double gyroXrate = (double)gyroX/131.0;
    double gyroYrate = -((double)gyroY/131.0);
    gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000); // Calculate gyro angle using the unbiased rate
    gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);
 
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); // Calculate the angle using a Kalman filter
    kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);
    timer = micros();
 
    /*
    Serial.println();
    Serial.print("X:");
    Serial.print(kalAngleX,0);
    Serial.print(" ");
  
    Serial.print("Y:");
    Serial.print(kalAngleY,0);
    Serial.println(" ");
    */
    
    //... здесь помещается блок отправки данных
    client.publish(sTopic_3,String(Counter)); // отправляем в топик для счётчика его значение
    Serial.println("Counter:" + String(Counter, DEC)); // эхо в порт
    
    client.publish(sTopic_5,String(kalAngleX)); // отправляем в топик для акселерометра значение координаты X
    Serial.println("GiroX:" + String(kalAngleX)); // эхо в порт
    
    client.publish(sTopic_6,String(kalAngleY)); // отправляем в топик для акселерометра значение координаты Y
    Serial.println("GiroY:" + String(kalAngleY)); // эхо в порт
    
    //Реализация счётчика публикаций
    Counter++;
    
    tm = DATA_REPEATE; // установка пауза между публикациями данных, единица измерения 10 мс
  }
  tm--;
  delay(10);//Данное значение уменьшать категорически не рекомедуется!!!
  
}//End of void DataSend()

void i2cWrite(uint8_t registerAddress, uint8_t data)
{
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission(); // Send stop
}

uint8_t* i2cRead(uint8_t registerAddress, uint8_t nbytes) 
{
  uint8_t data[nbytes];
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false); // Don't release the bus
  Wire.requestFrom(IMUAddress, nbytes); // Send a repeated start and then release the bus after reading
  for(uint8_t i = 0; i < nbytes; i++)
    data [i]= Wire.read();
  return data;
}
