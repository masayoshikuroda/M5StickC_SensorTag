#include <M5StickC.h>
#include "DHT12.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "bmm150.h"
#include "bmm150_defs.h"
#include <MadgwickAHRS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2904.h>

DHT12 dht12;
float dht12Temperature;
float dht12Humidity;

bool DHT12ReadData() {
  dht12Temperature = dht12.readTemperature();
  dht12Humidity    = dht12.readHumidity();
  return true;
};

Adafruit_BMP280 bme;
float bmePressure;

bool BMP280ReadData() {
  bmePressure = bme.readPressure(); 
  return true;
};

BMM150 bmm = BMM150();
bmm150_mag_data value_offset;
float magX, magY, magZ;

void calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();  
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();
  
  while((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();
    
    /* Update x-Axis max/min value */
    if(value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    } 
    else if(value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if(value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    } 
    else if(value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if(value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    } 
    else if(value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }
    
    Serial.print(".");
    delay(1);

  }

  value_offset.x = value_x_min + (value_x_max - value_x_min)/2;
  value_offset.y = value_y_min + (value_y_max - value_y_min)/2;
  value_offset.z = value_z_min + (value_z_max - value_z_min)/2;
}

bool BMM150ReadData() {
  bmm.read_mag_data();
  bmm150_mag_data m;
  magX = bmm.raw_mag_data.raw_datax - value_offset.x;
  magY = bmm.raw_mag_data.raw_datay - value_offset.y;
  magZ = bmm.raw_mag_data.raw_dataz - value_offset.z;
  return true; 
}

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float pitch, roll, yaw;
float imuTemp;

bool MPU6886ReadData() {
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getTempData(&imuTemp);
  M5.IMU.getAhrsData(&pitch,&roll,&yaw);
  return true; 
}

class ServerCallbacks: public BLEServerCallbacks {
 public:
    bool* _pConnected;

    ServerCallbacks(bool* connected) {
      _pConnected = connected;
    }
    void onConnect(BLEServer* pServer) {
      *_pConnected = true;
      M5.Lcd.println("ServerCallbacks onConnect");
      Serial.println("ServerCallbacks onConnect");
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(0, 0);
    }
    void onDisconnect(BLEServer* pServer) {
      *_pConnected = false;
      M5.Lcd.println("ServerCallbacks onDisconnect");
      Serial.println("ServerCallbacks onDisconnect");
    }
};

BLEServer* createServer(char* name, bool* pConnected) {
  BLEDevice::init(name);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks(pConnected));  
};

std::string toMACAddrString(char* buf, uint8_t* mac) {
  size_t len = sprintf(buf, " %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]); 
  return std::string(buf, len);
};

std::string toUInt8String(char* buf, uint8_t v) {
  size_t len = sprintf(buf, " %04d", v); 
  return std::string(buf, len);  
};

class InformationCharacteristic : public BLECharacteristic {
public:
  InformationCharacteristic(BLEUUID uuid, std::string value) : BLECharacteristic(uuid) {
    this->setReadProperty(true);
    this->setValue(value);
  }
};

BLEService* createInformationService(BLEServer* pServer) {
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x180a));
  uint8_t mac[6];
  char buf[256];
  esp_efuse_mac_get_default(mac);
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a23), "System ID"));
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a25), toMACAddrString(buf, mac)));
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a26), esp_get_idf_version()));
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a27), toUInt8String(buf, ESP.getChipRevision())));
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a28), "Software Rev"));
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a29), "M5Stack")); 
  return pService;
};

class BatteryLevelCallbacks: public BLECharacteristicCallbacks {
void onRead(BLECharacteristic *pCharacteristic) {
  }
  void onNotify(BLECharacteristic *pCharacteristic) {
    onRead(pCharacteristic);
  }
};

class BatteryLevelCharacteristic : public BLECharacteristic {
public:
  BatteryLevelCharacteristic(BLECharacteristicCallbacks* pCallbacks) : BLECharacteristic(BLEUUID((uint16_t)0x2a19)) {
    this->setCallbacks(pCallbacks);    
    this->setReadProperty(true);
    this->setNotifyProperty(true);
     
    BLE2904 *pDescriptor = new BLE2904();
    pDescriptor->setFormat(BLE2904::FORMAT_UINT8);
    pDescriptor->setNamespace(1);
    pDescriptor->setUnit(0x27ad);               
    this->addDescriptor(pDescriptor);
  }
};

BLEService* createBatteryService(BLEServer* pServer, BLECharacteristic* pCharacteristic) {
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x180F));
  pService->addCharacteristic(pCharacteristic);
  return pService;
};

class SimpleKeysCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic) {
    uint8_t value = 0x00;
    value |= M5.BtnA.wasReleased() ? 0x01 : 0x00;
    value |= M5.BtnB.wasReleased() ? 0x02 : 0x00;
    pCharacteristic->setValue(&value, 1);
  }
  void onNotify(BLECharacteristic *pCharacteristic) {
    onRead(pCharacteristic);
    uint8_t value = *pCharacteristic->getData();
    if (value != 0x00) {
      String msg = "Button was released";
      M5.Lcd.println(msg);
      Serial.println(msg);
    }   
  }
};

class SimpleKeysCharacteristic : public BLECharacteristic {
public:
  SimpleKeysCharacteristic(BLECharacteristicCallbacks* pCallbacks) : BLECharacteristic(BLEUUID((uint16_t)0xffe1)) {
    this->setReadProperty(true);
    this->setNotifyProperty(true);
    this->setCallbacks(pCallbacks);
  }
};

BLEService* createSimpleKeysService(BLEServer* pServer, BLECharacteristic* pCharacteristic) {
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0xffe0));
  pService->addCharacteristic(pCharacteristic);
  return pService;
};

class SensorCharacteristic : public BLECharacteristic {
private:
  int lastUpdate;
  void updateTime() {
    this->lastUpdate = millis();
  }
public:
  SensorCharacteristic(const char* uuid) : BLECharacteristic(uuid) {
    this->setReadProperty(true);
    this->setNotifyProperty(true);
  }
  SensorCharacteristic(const char* uuid, BLECharacteristicCallbacks* pCallbacks) : SensorCharacteristic(uuid) {
    this->setCallbacks(pCallbacks);
  }
  bool elapsed(int msecInterval) {
    int current = millis();
    bool ret = (current - this->lastUpdate) > msecInterval;
    return ret;
  }
  void setValue(uint8_t* data, size_t size) {
    this->BLECharacteristic::setValue(data, size);
    this->updateTime();
  }
  void setValue(uint8_t value) {
    this->BLECharacteristic::setValue(&value, 1);
    this->updateTime();
  }
  void setValue(uint16_t value) {
    this->BLECharacteristic::setValue(value);
    this->updateTime();
  }
  void setValue(uint32_t value) {
    this->BLECharacteristic::setValue(value);
    this->updateTime();
  }
};

class UInt8ConfigCharacteristic : public BLECharacteristic {
  int lastUpdate;
public:
  UInt8ConfigCharacteristic(const char* uuid, uint8_t value) : BLECharacteristic(uuid) {
    this->setReadProperty(true);
    this->setWriteProperty(true);
    this->setValue(&value, 1);   
  }
};

class PeriodCharacteristic : public UInt8ConfigCharacteristic {
public:
  PeriodCharacteristic(const char* uuid, uint8_t value) : UInt8ConfigCharacteristic(uuid, value) {
  }
  uint8_t getPeriod() { // 10msec
    return this->getData()[0];
  }
};

class UInt16ConfigCharacteristic : public BLECharacteristic {
public:
  UInt16ConfigCharacteristic(const char* uuid, uint16_t initialValue) : BLECharacteristic(uuid) {
    this->setReadProperty(true);
    this->setWriteProperty(true);
    this->setValue(initialValue);
  }
};

const char* temperature_sensor = "f000aa00-0451-4000-b000-000000000000";
const char* temperature_data   = "f000aa01-0451-4000-b000-000000000000";
const char* temperature_config = "f000aa02-0451-4000-b000-000000000000";
const char* temperature_period = "f000aa03-0451-4000-b000-000000000000";

BLEService* createTemperatureService(BLEServer* pServer, BLECharacteristic* pData, BLECharacteristic* pPeriod) {
  BLEService* pService = pServer->createService(temperature_sensor);
  pService->addCharacteristic(pData);
  pService->addCharacteristic(new UInt8ConfigCharacteristic(temperature_config, 0x01));
  pService->addCharacteristic(pPeriod);
  return pService;  
};

BLEService* createTemperatureService(BLEServer* pServer, BLECharacteristic* pData) {
    BLECharacteristic* pPeriod = new PeriodCharacteristic(temperature_period, 0x64);
    return createTemperatureService(pServer, pData, pPeriod);
};

const char* movement_sensor = "f000aa80-0451-4000-b000-000000000000";
const char* movement_data   = "f000aa81-0451-4000-b000-000000000000";
const char* movement_config = "f000aa82-0451-4000-b000-000000000000";
const char* movement_period = "f000aa83-0451-4000-b000-000000000000";

BLEService* createMovementService(BLEServer* pServer, BLECharacteristic* pData, BLECharacteristic* pPeriod) {
  BLEService* pService = pServer->createService(movement_sensor);
  pService->addCharacteristic(pData);
  pService->addCharacteristic(new UInt16ConfigCharacteristic(movement_config, 0x00ff));
  pService->addCharacteristic(pPeriod);
  return pService;
};

BLEService* createMovementService(BLEServer* pServer, BLECharacteristic* pData) {
  BLECharacteristic* pPeriod = new PeriodCharacteristic(movement_period, 0x64);
  return createMovementService(pServer, pData, pPeriod);
};

const char* humidity_sensor = "f000aa20-0451-4000-b000-000000000000";
const char* humidity_data   = "f000aa21-0451-4000-b000-000000000000";
const char* humidity_config = "f000aa22-0451-4000-b000-000000000000";
const char* humidity_period = "f000aa23-0451-4000-b000-000000000000";

BLEService* createHumidityService(BLEServer* pServer, BLECharacteristic* pData, BLECharacteristic* pPeriod) {
  BLEService* pService = pServer->createService(humidity_sensor);
  pService->addCharacteristic(pData);
  pService->addCharacteristic(new UInt8ConfigCharacteristic(humidity_config, 0x01));
  pService->addCharacteristic(pPeriod);
  return pService; 
};

BLEService* createHumidityService(BLEServer* pServer, BLECharacteristic* pData) {
  BLECharacteristic* pPeriod = new PeriodCharacteristic(humidity_period, 0x1e);
  return createHumidityService(pServer, pData, pPeriod);
};

const char* barometer_sensor = "f000aa40-0451-4000-b000-000000000000";
const char* barometer_data   = "f000aa41-0451-4000-b000-000000000000";
const char* barometer_config = "f000aa42-0451-4000-b000-000000000000";
const char* barometer_period = "f000aa44-0451-4000-b000-000000000000";

BLEService* createBarometerService(BLEServer* pServer, BLECharacteristic* pData, BLECharacteristic* pPeriod) {
  BLEService* pService = pServer->createService(barometer_sensor);
  pService->addCharacteristic(pData);
  pService->addCharacteristic(new UInt8ConfigCharacteristic(barometer_config, 0x00));
  pService->addCharacteristic(pPeriod);
  return pService;
};

BLEService* createBarometerService(BLEServer* pServer, BLECharacteristic* pData) {
  BLECharacteristic* pPeriod = new PeriodCharacteristic(humidity_period, 0x1e);
  return createBarometerService(pServer, pData, pPeriod);  
};

const char* io_service = "f000aa64-0451-4000-b000-000000000000";
const char* io_data    = "f000aa65-0451-4000-b000-000000000000";
const char* io_config  = "f000aa66-0451-4000-b000-000000000000";

class IOCallbacks: public BLECharacteristicCallbacks {
private:
  bool _useLED;
  bool _useSpeaker;
public:
  IOCallbacks(bool useLED, bool useSpeaker) {
    this->_useLED = useLED;
    this->_useSpeaker = useSpeaker;
  }
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t value = pCharacteristic->getValue()[0];
    Serial.print(value);
    Serial.println("");
    if (this->_useLED) {
      bool red = (value & 0x01) > 0;
      if (red) {
        digitalWrite(M5_LED, LOW);  
        M5.Lcd.println("Red LED on");
        Serial.println("Red LED on");
      } else {
        digitalWrite(M5_LED, HIGH);  
        M5.Lcd.println("Red LED off");
        Serial.println("Red LED off");         
      }
    }
    if (this->_useSpeaker) {
      bool buzzer = (value & 0x04) > 0;
      if (buzzer) {
        ledcWriteTone(0, 440);
        M5.Lcd.println("Buzzer on");
        Serial.println("Buzzer on");      
      } else {
        ledcWriteTone(0, 0);
        M5.Lcd.println("Buzzer off");
        Serial.println("Buzzer off");         
      }
    }
  }
};

class IOCharacteristic : public BLECharacteristic {
public:
  IOCharacteristic(BLECharacteristicCallbacks* pCallbacks) : BLECharacteristic(BLEUUID(io_data)) {
    this->setCallbacks(pCallbacks);
    this->setReadProperty(true);
    this->setWriteProperty(true);
    uint8_t value = 0x00;
    this->setValue(&value, 1);
  }
};

BLEService* createIOService(BLEServer* pServer, bool useLED, bool useSpeaker) {
  BLEService* pService = pServer->createService(BLEUUID(io_service));
  pService->addCharacteristic(new IOCharacteristic(new IOCallbacks(useLED, useSpeaker)));
  pService->addCharacteristic(new UInt8ConfigCharacteristic(io_config, 0x01));
  return pService;
}

bool isDHT12 = false;
bool isBME280 = false;
bool isBMM150 = false;
bool isIMU = true;
bool isLED = true;
bool isSpeaker = false;

bool deviceConnected = false;
BLECharacteristic* pBatteryLevelCharacteristic = NULL;
BLECharacteristic* pSimpleKeysCharacteristic   = NULL;
SensorCharacteristic* pTemperatureData         = NULL;
PeriodCharacteristic* pTemperaturePeriod       = NULL;
SensorCharacteristic* pMovementData            = NULL;
PeriodCharacteristic* pMovementPeriod          = NULL;
SensorCharacteristic* pHumidityData            = NULL;
PeriodCharacteristic* pHumidityPeriod          = NULL;
SensorCharacteristic* pBarometerData           = NULL;
PeriodCharacteristic* pBarometerPeriod         = NULL;

void setup() {
  M5.begin();
  
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);
  
  M5.Lcd.println("enter setup");
  Serial.println("enter setup");

  if (isBME280 || isBMM150 || isDHT12) {
    Wire.begin(0,26);
  }
  
  if (isBME280) {
    while (!bme.begin(0x76)){  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      M5.Lcd.println("Could not find a valid BMP280 sensor, check wiring!");
    }
  }

  if (isBMM150) {
    while (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
      Serial.println("Chip ID of BMM150 can not read!");
      M5.Lcd.println("Chip ID of BMM150 can not read!");
    }
  }
  calibrate(10);    

  if (isIMU) {
    M5.IMU.Init();
  }
  
  BLEServer *pServer = createServer("M5StickC", &deviceConnected);

  BLEService* pService = createInformationService(pServer);
  pService->addCharacteristic(new InformationCharacteristic(BLEUUID((uint16_t)0x2a24), "M5StickC"));
  pService->start();
  M5.Lcd.println("Information Service start");
  Serial.println("Information Service start");

  pBatteryLevelCharacteristic = new BatteryLevelCharacteristic(new BatteryLevelCallbacks());
  createBatteryService(pServer, pBatteryLevelCharacteristic)->start();
  M5.Lcd.println("Battery Service start");
  Serial.println("Battery Service start");

  pSimpleKeysCharacteristic = new SimpleKeysCharacteristic(new SimpleKeysCallbacks());
  createSimpleKeysService(pServer, pSimpleKeysCharacteristic)->start();
  M5.Lcd.println("SimpleKeys Service start");
  Serial.println("SimpleKeys Service start");
 
  if (isDHT12) {
    pTemperatureData = new SensorCharacteristic(temperature_data);
    pTemperaturePeriod = new PeriodCharacteristic(temperature_period, 0x64);
    createTemperatureService(pServer, pTemperatureData, pTemperaturePeriod)->start();
    M5.Lcd.println("Temperature Service start");
    Serial.println("Temperature Service start");
  }
  
  if (isIMU) {
    pMovementData = new SensorCharacteristic(movement_data);
    pMovementPeriod = new PeriodCharacteristic(movement_period, 0x64);
    createMovementService(pServer, pMovementData, pMovementPeriod)->start();
    M5.Lcd.println("Movement Service start");
    Serial.println("Movement Service start");   
  }
  
  if (isDHT12) {
    pHumidityData = new SensorCharacteristic(humidity_sensor);
    pHumidityPeriod = new PeriodCharacteristic(humidity_period, 0x64);
    createHumidityService(pServer, pHumidityData, pHumidityPeriod)->start();
    M5.Lcd.println("Humidity Service start");
    Serial.println("Humidity Service start");
  }

  if (isBME280) {
    pBarometerData = new SensorCharacteristic(barometer_data);
    pBarometerPeriod = new PeriodCharacteristic(barometer_period, 0x64);
    createBarometerService(pServer, pBarometerData, pBarometerPeriod)->start();
    M5.Lcd.println("Barometer Service start");
    Serial.println("Barometer Service start");
  }

  if (isLED) {
    pinMode(M5_LED, OUTPUT);
    digitalWrite(M5_LED, HIGH);
  }
  
  if (isSpeaker) {
    const int servo_pin = 26;
    int freq = 50;
    int ledChannel = 0;
    int resolution = 10;
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(servo_pin, ledChannel);
    ledcWrite(ledChannel, 256);
  }
  if (isLED || isSpeaker) {
    createIOService(pServer, isLED, isSpeaker)->start();
    M5.Lcd.println("IO Service start");
    Serial.println("IO Service start"); 
  }
  
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  M5.Lcd.println("Advertising start");
  Serial.println("Advertising start");
  
  M5.Lcd.println("exit setup");
  Serial.println("exit setup");
}

void loop() {
  M5.Lcd.setCursor(0, 0);
  
  float bat_power = M5.Axp.GetBatVoltage();
  Serial.printf("Batt Power[V]: %1.2f\r\n", bat_power);
  M5.Lcd.printf("Batt Power[V]: %1.2f\r\n", bat_power);
  pBatteryLevelCharacteristic->setValue(bat_power);

  pSimpleKeysCharacteristic->notify();
  
  if (isDHT12) {
    DHT12ReadData();
    Serial.printf("Temperature[degC]: %2.2f\r\n", dht12Temperature);
    M5.Lcd.printf("Temperature[degC]: %2.2f\r\n", dht12Temperature);
    Serial.printf("Humidity[%%]:      %0.2f\r\n", dht12Humidity);
    M5.Lcd.printf("Humidity[%%]:      %0.2f\r\n", dht12Humidity);
       
    if ((pTemperatureData != NULL) && (pTemperatureData->elapsed(pTemperaturePeriod->getPeriod()*10) > 0)) {
      uint16_t value = (int)(dht12Temperature / 0.03125) << 2;
      pTemperatureData->setValue(value);
      pTemperatureData->notify(value);
    }
    if ((pHumidityData != NULL) && (pHumidityData->elapsed(pHumidityPeriod->getPeriod()*10) > 0)) {
      int16_t value[2];
      value[0] = (dht12Temperature + 40)*65536 /165;
      value[1] = dht12Humidity*65536/100;
      pHumidityData->setValue((uint8_t*)value, 4);
      pHumidityData->notify();
    }
  }

  if (isBME280) {
    BMP280ReadData();
    Serial.printf("Pressure[Pa]: %0.2f\r\n", bmePressure);
    M5.Lcd.printf("Pressure[Pa]: %0.2f\r\n", bmePressure);
     
    if ((pBarometerData != NULL) && (pBarometerData->elapsed(pBarometerPeriod->getPeriod()*10) > 0)) {
      uint32_t tInt = dht12Temperature*100;
      uint8_t* t = (uint8_t*)&tInt;
      uint32_t pInt = bmePressure;
      uint8_t* p = (uint8_t*)&pInt;
      uint8_t value[6];
      value[0] = *(t + 0);
      value[1] = *(t + 1);
      value[2] = *(t + 2);
      value[3] = *(p + 0);
      value[4] = *(p + 1);
      value[5] = *(p + 2);
      pBarometerData->setValue(value, 6);   
      pBarometerData->notify();   
    }
  }

  if (isIMU) {
    MPU6886ReadData();
    Serial.printf("A: %5.1f  %5.1f  %5.1f\r\n", accX,accY,accZ);
    M5.Lcd.printf("A: %5.1f  %5.1f  %5.1f\r\n", accX,accY,accZ);
    Serial.printf("G: %5.1f  %5.1f  %5.1f\r\n", gyroX,gyroY,gyroZ);
    M5.Lcd.printf("G: %5.1f  %5.1f  %5.1f\r\n", gyroX,gyroY,gyroZ);
  }
  if (isBMM150) {
    BMM150ReadData();
    Serial.printf("M: %5.1f  %5.1f  %5.1f\r\n", magX,magY,magZ);
    M5.Lcd.printf("M: %5.1f  %5.1f  %5.1f\r\n", magX,magY,magZ);
  }

  if ((pMovementData != NULL) && pMovementData->elapsed(pMovementPeriod->getPeriod()*10)) {
    int16_t value[9];
    value[0] = (int16_t)(gyroX*65536/500);
    value[1] = (int16_t)(gyroY*65536/500);
    value[2] = (int16_t)(gyroZ*65536/500);
    value[3] = (int16_t)(accX*32768/8);
    value[4] = (int16_t)(accY*32768/8);
    value[5] = (int16_t)(accZ*32768/8);
    value[6] = (int16_t)(magX/10*8190/32768);
    value[7] = (int16_t)(magY/10*8190/32768);
    value[8] = (int16_t)(magZ/10*8190/32768);      
    pMovementData->setValue((unsigned char*)value, sizeof(value));
    pMovementData->notify();    
  }

  if (M5.BtnA.wasPressed()) {
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(0, 0);
  }
  
  delay(100);
  M5.update();
}
