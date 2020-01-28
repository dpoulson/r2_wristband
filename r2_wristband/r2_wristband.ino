#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>
#include "BLEDevice.h"

// Images/Icons
#include "charge.h"
#include "r2_logo.h"
#include "icons.h"
#include "wifi.h"
#include "internet.h"

#include "esp_adc_cal.h"

#define TP_PIN_PIN          33
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define IMU_INT_PIN         38
#define RTC_INT_PIN         34
#define BATT_ADC_PIN        35
#define VBUS_PIN            36
#define TP_PWR_PIN          25
#define LED_PIN             4
#define CHARGE_PIN          32

TFT_eSPI tft = TFT_eSPI();

char buff[256];
bool rtcIrq = false;
bool initial = 1;
bool otaStart = false;
bool pressed = false;

uint8_t func_select = 0;
uint32_t targetTime = 0;       // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;

uint32_t pressedTime = 0;
bool charge_indication = false;

const char* ssid     = "60 Chequers Avenue AP1";
const char* password = "trial3211";

//char droid_name[16];
std::string droid_name;
int wifi, internet;
int volume,main_bat,controller_bat;

#define LONG_PRESS 3000
#define SLEEP_TIME 20000
#define POLL_INTERVAL 1000

// ********* BLE Settings
// The remote service we wish to connect to.
static BLEUUID   serviceUUID("0000fff0-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID   charMainBatteryUUID("0000fff2-0000-1000-8000-00805f9b34fb");
static BLEUUID   charCtrlBatteryUUID("0000fff3-0000-1000-8000-00805f9b34fb");
static BLEUUID   charVolumeUUID("0000fff4-0000-1000-8000-00805f9b34fb");
static BLEUUID   charDroidNameUUID("0000fff1-0000-1000-8000-00805f9b34fb");


static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pMainBattery;
static BLERemoteCharacteristic* pCtrlBattery;
static BLERemoteCharacteristic* pVolume;
static BLERemoteCharacteristic* pDroidName;
static BLEAdvertisedDevice* myDevice;

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pMainBattery = pRemoteService->getCharacteristic(charMainBatteryUUID);
    pCtrlBattery = pRemoteService->getCharacteristic(charCtrlBatteryUUID);
    pVolume = pRemoteService->getCharacteristic(charVolumeUUID);
    pDroidName = pRemoteService->getCharacteristic(charDroidNameUUID);

    if (pDroidName == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charDroidNameUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    main_bat = pMainBattery->readUInt8();
    controller_bat = pCtrlBattery->readUInt8();
    volume = pVolume->readUInt8();
    droid_name = pDroidName->readValue();
    Serial.print("Droid Name: ");
    Serial.println(droid_name.c_str());
    Serial.print("Main Battery: ");
    Serial.println(main_bat);
    Serial.print("CTRL Battery: ");
    Serial.println(controller_bat);
    Serial.print("Volume: ");
    Serial.println(volume);
    connected = true;
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    pressedTime = millis();
    
    tft.init();
    tft.setRotation(0);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK); // Note: the new fonts do not draw the background colour
    tft.pushImage(3, 3, 32, 32, r2bglogosml);
    
    tft.pushImage(0,52,16,16,battery);
    tft.pushImage(0,70,16,16,controller);
    tft.pushImage(0,88,16,16,speaker);

    pinMode(TP_PIN_PIN, INPUT);
    //! Must be set to pull-up output mode in order to wake up in deep sleep mode
    pinMode(TP_PWR_PIN, PULLUP);
    digitalWrite(TP_PWR_PIN, HIGH);

    pinMode(LED_PIN, OUTPUT);

    pinMode(CHARGE_PIN, INPUT_PULLUP);
    attachInterrupt(CHARGE_PIN, [] {
        charge_indication = true;
    }, CHANGE);

    if (digitalRead(CHARGE_PIN) == LOW) {
        charge_indication = true;
    }
      Serial.println("Starting Arduino BLE Client application...");
    BLEDevice::init("");

    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device.  Specify that we want active scanning and start the
    // scan to run for 5 seconds.
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
    // If the flag "doConnect" is true then we have scanned for and found the desired
    // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
    // connected we set the connected flag to be true.
    if (doConnect == true) {
      if (connectToServer()) {
        Serial.println("We are now connected to the BLE Server.");
      } else {
        Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      }
      doConnect = false;
    }

    // If we are connected to a peer BLE Server, update the characteristic each time we are reached
    // with the current time since boot.
    if (connected) {
      Serial.println("Polling....");
    }
    tft.drawString(droid_name.c_str(),0,40);
    if(wifi == 1) {
      tft.pushImage(40,3,16,16,wifi_on);
     } else {
      tft.pushImage(40,3,16,16,wifi_off);
     }

     if(internet == 1) {
      tft.pushImage(60,3,16,16,internet_on);
     } else {
      tft.pushImage(60,3,16,16,internet_off);
     }

     // Main Battery
     drawProgressBar(20, 52, 55, 14, main_bat, TFT_WHITE, TFT_BLUE);
     // Controller Battery
     drawProgressBar(20, 70, 55, 14, controller_bat, TFT_WHITE, TFT_BLUE);
     // Volume
     drawProgressBar(20, 88, 55, 14, volume, TFT_WHITE, TFT_BLUE);
}

void loop() {

    if (charge_indication) {
        charge_indication = false;
        if (digitalRead(CHARGE_PIN) == LOW) {
           tft.pushImage(0, 140, 34, 16, charge);
        } else {
           tft.fillRect(0, 140, 34, 16, TFT_BLACK);
        }
    }
    
   if (digitalRead(TP_PIN_PIN) == HIGH) {
        if (!pressed) {
            initial = 1;
            tft.fillScreen(TFT_BLACK);
            func_select = func_select + 1 > 2 ? 0 : func_select + 1;
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            pressed = true;
            pressedTime = millis();
        } else {
            if (millis() - pressedTime > LONG_PRESS) {
                tft.fillScreen(TFT_BLACK);
                tft.drawString("Reset WiFi Setting",  20, tft.height() / 2 );
                delay(3000);
                esp_restart();
            }
        }
    } else {
        pressed = false;
    }

    if (millis() - pressedTime > SLEEP_TIME) {
      func_select = 1;
    }

   switch (func_select) {
   case 0:

     break;

   case 1:
     // Turn off
     tft.writecommand(ST7735_SLPIN);
     tft.writecommand(ST7735_DISPOFF);
     esp_sleep_enable_ext1_wakeup(GPIO_SEL_33, ESP_EXT1_WAKEUP_ANY_HIGH);
     esp_deep_sleep_start();
     break;
   default:
     break;
   }
}

float getVoltage()
{
    uint16_t v = analogRead(BATT_ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return battery_voltage;
}


void setupADC()
{
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }
}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
    if (percentage == 0) {
        tft.fillRoundRect(x0, y0, w, h, 3, TFT_BLACK);
    }
    uint8_t margin = 2;
    uint16_t barHeight = h - 2 * margin;
    uint16_t barWidth = w - 2 * margin;
    tft.drawRoundRect(x0, y0, w, h, 3, frameColor);
    tft.fillRect(x0 + margin, y0 + margin, barWidth * percentage / 100.0, barHeight, barColor);
}
