// #include <Adafruit_Sensor.h>
// #include <DHT.h>
// #include <DHT_U.h>
#include <ArduinoJson.h>
#include <ArduinoMqttClient.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEExceptions.h>

// #include "esp_camera.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "OV2640Streamer.h"
#include "OV2640.h"
#include "CRtspSession.h"
#include <WebServer.h>
#include "credentials.h"

// Camera stuff
#define USEBOARD_AITHINKER
#define ENABLE_RTSPSERVER
#define RESOLUTION  FRAMESIZE_SVGA // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
#define QUALITY        20          // JPEG quality 10-63 (lower means better quality)
OV2640 cam;

#ifdef ENABLE_RTSPSERVER
WiFiServer rtspServer(8554);
CStreamer *streamer;
//CRtspSession *session;
#endif

// BLE stuff
// name of the Inkbird bluetooth temp sensor
#define bleServerName "tps"
// the service UUID is FFF0
static BLEUUID bmeServiceUUID("FFF0");
// the temperature characteristic UUID is FFF2
static BLEUUID temperatureCharacteristicUUID("FFF2");
//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
//Characteristicd that we want to read
static BLERemoteCharacteristic* temperatureCharacteristic;
//Flags stating if should begin connecting and if the connection is up
static bool ble_doConnect = true;
static bool ble_connected = false;
static bool ble_temperature_updated = false;
static float tps_temperature_f = 0;
static float tps_temperature_c = 0;
const long bte_interval = 900000;
unsigned long bte_previousMillis = 0;
BLEClient* pClient = BLEDevice::createClient();

// Wifi and MQTT stuff
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

int port     = 1883;
std::string topic_base = "home/sensor/";
std::string SENSOR_NAME = "";

void initializeSystem() {
  Serial.setDebugOutput(true);
}

// initializers
void setECUID()
{
  uint32_t low = ESP.getEfuseMac() & 0xFFFFFFFF;
  uint32_t high = ( ESP.getEfuseMac() >> 32 ) % 0xFFFFFFFF;
  Serial.print(low);
  Serial.print(" : ");
  Serial.println(high);

  SENSOR_NAME = "0x" + std::to_string(low) + std::to_string(high);
  Serial.print("SENSOR_NAME: ");
  Serial.println(SENSOR_NAME.c_str());
}

void setupMQTT() {
  mqttClient.setUsernamePassword(mqttuser, mqttpass);
}

void connectToWIFI() {
  IPAddress ip;
  // attempt to connect to Wifi network:
  Serial.print("WIFI - Attempting to connect to WPA SSID ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.print("WIFI - Sucessfully connected to SSID ");
  Serial.println(ssid);
  ip = WiFi.localIP();
  Serial.print("WIFI - IP address: ");
  Serial.println(ip);
}

void connectToMQTT() {
  Serial.print("MQTT - Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT - MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }
  Serial.println("MQTT - You're connected to the MQTT broker!");
  
}

bool connectToBLE(BLEAddress pAddress) {

  // Connect to the remove BLE Server.
  bool ble_result = pClient->connect(pAddress);
  Serial.println("BLE - Connected to server");
  Serial.print("BLE - Connected to server: ");
  Serial.println(ble_result);
  if (!ble_result) {
    Serial.println("BLE - Failed to connect to server");
    ble_doConnect = true;
    ble_connected = false;
  }
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("BLE - Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  temperatureCharacteristic = pRemoteService->getCharacteristic(temperatureCharacteristicUUID);

  if (temperatureCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println("BLE - Found characteristics for temperature");

  return true;
}

void disconnectBLE() {
  pClient->disconnect();
}


//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE - Advertised Device Address: ");
    Serial.println(advertisedDevice.getAddress().toString().c_str());
    Serial.print("BLE - Advertised Device Name: ");
    Serial.println(advertisedDevice.getName().c_str());
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      Serial.print("BLE Address: ");
      Serial.println(pServerAddress->toString().c_str());
      ble_doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

void setupCamera() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  camera_config_t cconfig;
  cconfig = esp32cam_aithinker_config;
  if (psramFound()) {
    cconfig.frame_size = RESOLUTION; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    cconfig.jpeg_quality = QUALITY;
    cconfig.fb_count = 2;
  } else {
    if (RESOLUTION>FRAMESIZE_SVGA) {
     cconfig.frame_size = FRAMESIZE_SVGA;
    }
    cconfig.jpeg_quality = 12;
    cconfig.fb_count = 1;
  }
  cam.init(cconfig);


#ifdef ENABLE_RTSPSERVER
  rtspServer.begin();
  streamer = new OV2640Streamer(cam);             // our streamer for UDP/TCP based RTP transport

#endif
}

void updateRTSPCamera() {

#ifdef ENABLE_RTSPSERVER
    uint32_t msecPerFrame = 100;
    static uint32_t lastimage = millis();

    // If we have an active client connection, just service that until gone
    streamer->handleRequests(0); // we don't use a timeout here,
    // instead we send only if we have new enough frames
    uint32_t now = millis();
    if (streamer->anySessions()) {
      if (now > lastimage + msecPerFrame || now < lastimage) { // handle clock rollover
        streamer->streamImage(now);
        lastimage = now;

        // check if we are overrunning our max frame rate
        now = millis();
        if (now > lastimage + msecPerFrame) {
          printf("warning exceeding max frame rate of %d ms\n", now - lastimage);
        }
      }
    }

    WiFiClient rtspClient = rtspServer.accept();
    if (rtspClient) {
      Serial.print("client: ");
      Serial.print(rtspClient.remoteIP());
      Serial.println();
      streamer->addSession(rtspClient);
    }

#endif
}

void readTemperature() {
  uint16_t temp = 0;
  try {
    temp = temperatureCharacteristic->readUInt16();
  } catch (BLEDisconnectedException e) {
    Serial.println("Failed to read temperature from BLE server.");
    ble_connected = false;
    ble_doConnect = true;
  }
  if (temp == 0) {
    Serial.println("Wrong temperature reading. Ignoring value and reconnecting to BTE device.");
    ble_connected = false;
    ble_doConnect = true;
  } else {
    float temp_f = ((float) temp)/100 * 1.8 + 32;
    float temp_c = ((float) temp)/100;
    if (temp_f != tps_temperature_f) {
      ble_temperature_updated = true;
      tps_temperature_f = round(temp_f*100)/100;
      tps_temperature_c = round(temp_c*100)/100;
      Serial.print("Inkbird temp rounded: ");
      Serial.printf("%f\n", tps_temperature_f);
    }
  }
}

// SETUP
void setup()
{
  initializeSystem();
  //Serial.begin(115200);
  Serial.begin(9600);

  // while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }
  
  setECUID();
  setupMQTT();
  connectToWIFI();
  connectToMQTT();
  setupCamera();

  //Init BLE device
  BLEDevice::init("");
  Serial.println("BLE - Device initialized");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.

  // skip the scanning as we alread know the address
  pServerAddress = new BLEAddress("49:22:02:11:04:93");
  // BLEScan* pBLEScan = BLEDevice::getScan();
  // Serial.println("BLE - Starting scan");
  // pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // pBLEScan->setActiveScan(true);
  // pBLEScan->start(30);
  // Serial.println("BLE - Scan started");
  // pBLEde
}

void loop()
{
 
  // MQTT stuff
  mqttClient.poll();
  updateRTSPCamera();

  // BLE Stuff
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (ble_doConnect == true) {
    if (connectToBLE(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      readTemperature();
      ble_connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    ble_doConnect = false;
  } else {
    ; // Do nothing
  }

  unsigned long currentMillis = millis();
  if (currentMillis - bte_previousMillis >= bte_interval) {
    // save the last time a message was sent
    bte_previousMillis = currentMillis;

    if (ble_connected == true) {
      
      
      readTemperature();
    }
  }

  //if new temperature readings are available, print them out
  // TODO: send via MQTT
  if (ble_temperature_updated){
    ble_temperature_updated = false;
    Serial.print("Inkbird Temperature: ");
    Serial.print(tps_temperature_f);
    Serial.println(" F");
    StaticJsonDocument<256> doc;
    doc["temperature_c"] = serialized(String(tps_temperature_c,2)); 
    doc["temperature_f"] = serialized(String(tps_temperature_f,2));
    char payload[128] = "";
    int b = serializeJson(doc, payload);
    Serial.print("bytes = ");
    Serial.println(b,DEC);

    // std::string payload =   //"{ \"temperature_f\": %c, \"temperature_c\": %c}", temp_f_str, temp_c_str;

    std::string topic = topic_base + SENSOR_NAME + "_tps" + "/temperature";
    Serial.print("Publishing to topic: ");
    Serial.println(topic.c_str());
    Serial.print("Payload: ");
    Serial.println(payload);
    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(topic.c_str());
    mqttClient.print(payload);
    mqttClient.endMessage();
    // mqttClient.publish(topic.c_str(), payload);
  }
}
