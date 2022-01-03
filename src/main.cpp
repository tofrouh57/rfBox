#include <Arduino.h>
#include <SoftwareSerial.h>
#include <CO2.h>
#include <vector>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "AHTxx.h"
#include "SSD1306Wire.h"

const char *ssid = "unifi";
const char *password = "3N3FXAI4RP";
const char *mqttServer = "192.168.0.55"; // mqtt server

AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); // sensor address, sensor type
SSD1306Wire display(0x3c, SDA, SCL);

WiFiClient espClient;
PubSubClient MQTTclient(espClient); // lib required for mqtt

unsigned long startTime = millis();

#define CO2_RX_PIN D7   //32 Rx pin which the MHZ19 Tx pin is attached to
#define CO2_TX_PIN D8   //33 Tx pin which the MHZ19 Rx pin is attached
#define RF_PIN D6  // esp32: 12

const bool c_setVerbose{false};

SoftwareSerial co2Serial(CO2_RX_PIN, CO2_TX_PIN); // define MH-Z19 RX TX D3 (GPIO0) and D4 (GPIO2)
CO2_sensor myco2sensor;

// for values

struct s_rawValue
{
  unsigned long millis;
  unsigned long rawValue;
};

struct s_publish
{
  int device;
  int measure;
  double value;
  int amount{1};
  unsigned long millis;
};

struct s_envValue
{
  char co2[20];
  char temp[20];
  char hum[20];
  char pres[20];
  char volt[20];
};

std::vector<s_rawValue> v_rawValues;
std::vector<s_publish> v_publish;
std::vector<s_envValue> v_envValues;
s_rawValue rawValuesHelper;
s_publish tmp_publish;

int rawCounter{0};

// lhs == iterator on values to publish. rhs = publishhelper with raw value
// check is value belongs to same transmission using millis suuposed a transmission lasts max 2 secs
// current raw value.miili shall be not greater than stored millis+2seconds
inline bool operator==(const s_publish &lhs, const s_publish &rhs)
{
  return lhs.device == rhs.device &&
         lhs.measure == rhs.measure &&
         lhs.value == rhs.value &&
         lhs.millis + 2000 > rhs.millis;
}
// end for values

// RF

std::vector<unsigned long> rfQueue;

struct HighLow
{
  uint8_t high;
  uint8_t low;
};

struct _Protocol
{
  /** base pulse length in microseconds, e.g. 350 */
  uint16_t pulseLength;
  HighLow syncFactor;
  HighLow zero;
  HighLow one;
  bool invertedSignal;
};
#define RCSWITCH_MAX_CHANGES 67
const unsigned int nSeparationLimit = 4300;
volatile unsigned long nReceivedValue = 0;
volatile unsigned int nReceivedBitlength = 0;
volatile unsigned int nReceivedDelay = 0;
volatile unsigned int nReceivedProtocol = 0;

unsigned int timings[RCSWITCH_MAX_CHANGES];
int nReceiveTolerance = 60;
static const  _Protocol proto[] = {
    {350, {1, 31}, {1, 3}, {3, 1}, false},    // protocol 1
    {650, {1, 10}, {1, 2}, {2, 1}, false},    // protocol 2
    {100, {30, 71}, {4, 11}, {9, 6}, false},  // protocol 3
    {380, {1, 6}, {1, 3}, {3, 1}, false},     // protocol 4
    {500, {6, 14}, {1, 2}, {2, 1}, false},    // protocol 5
    {450, {23, 1}, {1, 2}, {2, 1}, true},     // protocol 6 (HT6P20B)
    {150, {2, 62}, {1, 6}, {6, 1}, false},    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
    {200, {3, 130}, {7, 16}, {3, 16}, false}, // protocol 8 Conrad RS-200 RX
    {200, {130, 7}, {16, 7}, {16, 3}, true},  // protocol 9 Conrad RS-200 TX
    {365, {18, 1}, {3, 1}, {1, 3}, true},     // protocol 10 (1ByOne Doorbell)
    {270, {36, 1}, {1, 2}, {2, 1}, true},     // protocol 11 (HT12E)
    {320, {36, 1}, {1, 2}, {2, 1}, true}      // protocol 12 (SM5212)
};

enum
{
  numProto = sizeof(proto) / sizeof(proto[0])
};

/* helper function for the receiveProtocol method */
static inline unsigned int diff(int A, int B)
{
  return abs(A - B);
}

bool IRAM_ATTR receiveProtocol(const int p, unsigned int changeCount)
{
  const _Protocol &pro = proto[p - 1];

  unsigned long code = 0;
  // Assuming the longer pulse length is the pulse captured in timings[0]
  const unsigned int syncLengthInPulses = ((pro.syncFactor.low) > (pro.syncFactor.high)) ? (pro.syncFactor.low) : (pro.syncFactor.high);
  const unsigned int delay = timings[0] / syncLengthInPulses;
  const unsigned int delayTolerance = delay * nReceiveTolerance / 100;
  const unsigned int firstDataTiming = (pro.invertedSignal) ? (2) : (1);

  for (unsigned int i = firstDataTiming; i < changeCount - 1; i += 2)
  {
    code <<= 1;
    if (diff(timings[i], delay * pro.zero.high) < delayTolerance &&
        diff(timings[i + 1], delay * pro.zero.low) < delayTolerance)
    {
      // zero
    }
    else if (diff(timings[i], delay * pro.one.high) < delayTolerance &&
             diff(timings[i + 1], delay * pro.one.low) < delayTolerance)
    {
      // one
      code |= 1;
    }
    else
    {
      // Failed
      return false;
    }
  }

  if (changeCount > 7)
  { // ignore very short transmissions: no device sends them, so this must be noise
    nReceivedValue = code;
    nReceivedBitlength = (changeCount - 1) / 2;
    nReceivedDelay = delay;
    nReceivedProtocol = p;
    //    rfQueue.push_back(code);   // add value to vector

    return true;
  }

  return false;
}

void IRAM_ATTR handleInterrupt()
{
  static unsigned int changeCount = 0;
  static unsigned long lastTime = 0;
  static unsigned int repeatCount = 0;

  const long time = micros();
  const unsigned int duration = time - lastTime;

  if (duration > nSeparationLimit)
  {
    // A long stretch without signal level change occurred. This could
    // be the gap between two transmission.
    if ((repeatCount == 0) || (diff(duration, timings[0]) < 200))
    {
      repeatCount++;
      if (repeatCount == 2)
      {
        for (unsigned int i = 1; i <= numProto; i++)
        {
          if (receiveProtocol(i, changeCount))
          {
            // put value to vector
            if (nReceivedProtocol == 1)
            {
              rawValuesHelper.millis = millis();
              rawValuesHelper.rawValue = nReceivedValue;
              v_rawValues.push_back(rawValuesHelper);
            }
            break;
          }
        }
        repeatCount = 0;
      }
    }
    changeCount = 0;
  }

  // detect overflow
  if (changeCount >= RCSWITCH_MAX_CHANGES)
  {
    changeCount = 0;
    repeatCount = 0;
  }

  timings[changeCount++] = duration;
  lastTime = time;
}

unsigned long _millis{0};
unsigned int _nbEntries{0};
int l_millis{0};

void processRawValues(int l_nbEntries)
{

  for (int itRaw = 0; itRaw < l_nbEntries; itRaw++)
  {
    bool l_valueFound{false}; // set valueFound false to know if a new entry has to be created in vector publish
    s_publish publishHelper;
    publishHelper.device = v_rawValues[itRaw].rawValue >> 28;
    publishHelper.measure = ((v_rawValues[itRaw].rawValue & 0xF000000) >> 24);
    publishHelper.millis = (v_rawValues[itRaw].millis);
    switch (publishHelper.measure)
    {

    case 1: // temparature
    {
      publishHelper.value = (v_rawValues[itRaw].rawValue & 0xFFFFFF) / 100.0 - 40;
      break;
    }

    case 2: // humidity
    {
      publishHelper.value = (v_rawValues[itRaw].rawValue & 0xFFFFFF) / 100.0;
      break;
    }

    case 3: // pressure
    {
      publishHelper.value = (v_rawValues[itRaw].rawValue & 0xFFFFFF) / 100.0;
      break;
    }

    case 4: // voltage
    {
      publishHelper.value = (v_rawValues[itRaw].rawValue & 0xFFFFFF) / 1000.0;
      break;
    }
    }
    // loop on all records to be published
    for (std::vector<s_publish>::iterator itpub = v_publish.begin(); itpub != v_publish.end(); ++itpub)
    {
      // if same device and measure already exists within the same 2 seconds, add 1 to amount
      if ((*itpub) == publishHelper) // inline ==
      {
        (*itpub).amount++;
        l_valueFound = true;
        break;
      }
    }
    // if it£s the first time we see this device/measure, add it to the list
    if ((!l_valueFound) && (publishHelper.device > 0) && (publishHelper.device <= 2) && (publishHelper.measure > 0) && (publishHelper.measure <= 4))
      v_publish.push_back(publishHelper);
  }

  // delete processed raw values vector entries
  for (int itRaw = 0; itRaw < l_nbEntries; itRaw++)
  {
    v_rawValues.erase(v_rawValues.begin());
  }

  // check for valid values
  // compare and set value to 0 if a biger exists for same device / measure
  for (int i = 0; i < v_publish.size() - 1; i++)
  {
    if ((v_publish[i].device == v_publish[i + 1].device) &&
        (v_publish[i].measure == v_publish[i + 1].measure) &&
        (v_publish[i].millis + 2000 > v_publish[i + 1].millis) &&
        (v_publish[i].amount >= v_publish[i + 1].amount))

      v_publish[i + 1].amount = 0;

    if ((v_publish[i].device == v_publish[i + 1].device) &&
        (v_publish[i].measure == v_publish[i + 1].measure) &&
        (v_publish[i].millis + 2000 > v_publish[i + 1].millis) &&
        (v_publish[i].amount < v_publish[i + 1].amount))

      v_publish[i].amount = 0;
  }

  // publish all that has amount > 0
}

void initWiFi()
{
  WiFi.begin(ssid, password);
  display.drawString(0, 10, "wifi ..");
  display.display();
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  display.drawString(0, 10, "wifi .. OK");
  display.display();
}



bool publishTopic(char i_topic[40], char i_payload[40])
{
  //  while (!MQTTclient.connected())
  //  {

  Serial.print(millis());
  Serial.println(":  Connecting to MQTT...");

  if (MQTTclient.connect("ESP32Client"))
  {
    Serial.print(millis());
    Serial.println(":  connected");
    MQTTclient.publish(i_topic, i_payload);
    return true;
    
  }
  else
  {
    Serial.print(millis());
    display.drawString(0, 20, "MQTT .. failed");
    display.display();
    Serial.print(":  failed with state ");
    Serial.println(MQTTclient.state());
    return false;
  }
  //  }
}

void setup()
{


// Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Booting ...");
  display.display();

  Serial.begin(115200);


  initWiFi();



  // init CO2
  co2Serial.begin(9600);
  myco2sensor.begin(co2Serial, Serial, false);

  // init RF
  attachInterrupt(12, handleInterrupt, CHANGE);

  initWiFi();
  MQTTclient.setServer(mqttServer, 1883);

  while (aht10.begin() != true)
  {
    Serial.println(F("AHT1x not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    display.drawString(0, 20, "AHT .. failed");
    display.display();

    delay(5000);
  }

    display.drawString(0, 20, "AHT .. OK");
    display.display();
  Serial.println(F("AHT10 OK"));

  

  // init with 2 sensors
  s_envValue l_envValues;
  v_envValues.push_back(l_envValues);
  v_envValues.push_back(l_envValues);

pinMode(D3, INPUT_PULLUP);
pinMode(D4, INPUT_PULLUP);
pinMode(D5, INPUT_PULLUP);


}

int wifi_delay{30 * 1000};
unsigned long wifi_millis{millis() + wifi_delay};
int rf_delay{10 * 1000};
unsigned long rf_millis{millis() + rf_delay};
int co2_delay{3 * 60 * 1000};
unsigned long co2_millis{millis() + co2_delay};
int aht_delay{3 * 60 * 1000};
unsigned long aht_millis{millis() + aht_delay};
int disp_delay{10 * 1000};
unsigned long disp_millis{millis() + disp_delay};
bool _disp{false};




void loop()
{


  if (digitalRead(D3) == LOW) Serial.println("D3 pressed");
  if (digitalRead(D4) == LOW) Serial.println("D4 pressed");
  if (digitalRead(D5) == LOW) Serial.println("D5 pressed");



  if ((millis() - aht_millis) > aht_delay)
  {
    float ahtValue{0};
    s_publish publishHelper;

    aht_millis = millis();
    Serial.print(millis());
    Serial.print(":  reading AHT10 - temperatur. Result : ");
    ahtValue = aht10.readTemperature(); // read 6-bytes via I2C, takes 80 milliseconds
    Serial.println(ahtValue);

    publishHelper.millis = millis();
    publishHelper.device = 2;
    publishHelper.measure = 1;
    publishHelper.amount = 1;
    publishHelper.value = ahtValue;
    v_publish.push_back(publishHelper);

    Serial.print(millis());
    Serial.print(":  reading AHT10 - humidity. Result : ");
    ahtValue = aht10.readHumidity(AHTXX_USE_READ_DATA); // use 6-bytes from temperature reading, takes zero milliseconds!!!
    Serial.println(ahtValue);
    publishHelper.millis = millis();
    publishHelper.device = 2;
    publishHelper.measure = 2;
    publishHelper.amount = 1;
    publishHelper.value = ahtValue;
    v_publish.push_back(publishHelper);

    if (ahtValue == AHTXX_ERROR)
    {
      Serial.print(millis());
      Serial.print(":  AHTXX_ERROR occured. Reset ");

      if (aht10.softReset() == true)
        Serial.println(" SUCCESS"); // as the last chance to make it alive
      else
        Serial.println(" FAIL");
    }
  }

  if ((millis() - co2_millis) > co2_delay)
  {
    s_publish publishHelper;

    co2_millis = millis();
    Serial.print(millis());
    Serial.print(":  reading CO2. Result : ");
    int ppm_uart = myco2sensor.read();
    Serial.println(ppm_uart);

    publishHelper.millis = millis();
    publishHelper.device = 2;
    publishHelper.measure = 5;
    publishHelper.amount = 1;
    publishHelper.value = ppm_uart;
    v_publish.push_back(publishHelper);
  }

  if ((millis() - rf_millis) > rf_delay)
  {
    rf_millis = millis();
    unsigned int curEntries{v_rawValues.size()};
    if ((_nbEntries == curEntries) && (curEntries > 0))
    {
      processRawValues(_nbEntries); // proc needs l_nbentries!!!!!
      _nbEntries = 0;
    }
    else
    {
      _nbEntries = curEntries;
    }
  }

  if ((millis() - wifi_millis) > wifi_delay)
  {

    wifi_millis = millis();
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.print(millis());
      Serial.println(":  wifi not connected ");
      WiFi.reconnect();
    }
    else
    {
      s_publish publishHelper;
      Serial.print(millis());
      Serial.print(":   wifi connected. IP:  ");
      Serial.println(WiFi.localIP());
      Serial.print(millis());
      Serial.print(":  in loop / Wifi check, publish alive. result: ");

      publishHelper.millis = millis();
      publishHelper.device = 2;
      publishHelper.measure = 0;
      publishHelper.amount = 1;
      publishHelper.value = 1;
      v_publish.push_back(publishHelper);
    }
  }

  if ((millis() - disp_millis) > disp_delay)
  {
      disp_millis = millis();
    if (_disp)
    {
      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "    INTERIEUR     ");
      display.drawString(0, 15, v_envValues[1].temp);
      display.drawString(64, 15, v_envValues[1].hum);
      display.drawString(0, 31, v_envValues[1].co2);
      display.drawString(64, 31, "ppm");
      
      display.display();
    }
    else
    {

      display.clear();
      display.setFont(ArialMT_Plain_16);
      display.drawString(0, 0, "    EXTERIEUR     ");
      display.drawString(0, 15, v_envValues[0].temp);
      display.drawString(64, 15, v_envValues[0].hum);
      display.drawString(0, 31, v_envValues[0].pres);
      display.drawString(64, 31, "hPa");
      display.drawString(0, 47, v_envValues[0].volt);
      display.drawString(64, 47, "V");
      display.display();
    }
    _disp = !_disp;
  }

  if ((v_publish.size() > 0) && (WiFi.status() == WL_CONNECTED))

  {
    for (std::vector<s_publish>::iterator itpub = v_publish.begin(); itpub != v_publish.end(); ++itpub)
    {
      if ((*itpub).amount == 0)
        continue;

      char l_topic[40];
      char l_payload[40];
      char l_string[40];
      char buffer[40];
      // publish to mqtt and replace publishhelper content with current vector value
      Serial.print(millis());
      Serial.print(":  ----- publier ");
      Serial.print(" ");
      Serial.print((*itpub).amount);
      Serial.print(" ");
      Serial.print((*itpub).millis);
      Serial.print(" ");
      Serial.print((*itpub).device);
      Serial.print(" ");
      Serial.print((*itpub).measure);
      Serial.print(" ");
      Serial.print((*itpub).value);
      Serial.print(" / ");
      Serial.println();
      sprintf(buffer, "%4.2f", (*itpub).value);
      strcpy(l_payload , buffer);
      strcpy(l_topic, "");
      strcat(l_topic, "environment");
      strcat(l_topic, "/");
      itoa(((*itpub).device) + 50, buffer, 10);
      strcat(l_topic, buffer);
      strcat(l_topic, "/");

      switch ((*itpub).measure)
      {
      case 0:
      {
      strcat(l_topic, "alive");
        break;
      }
      case 1:
      {
      strcat(l_topic, "temp");
        strcpy(v_envValues[(*itpub).device - 1].temp, "");
        strcat(v_envValues[(*itpub).device - 1].temp, l_payload);
        strcat(v_envValues[(*itpub).device - 1].temp, "°");

        break;
      }
      case 2:
      {
      strcat(l_topic, "hum");
        strcpy(v_envValues[(*itpub).device - 1].hum, "");
        strcat(v_envValues[(*itpub).device - 1].hum, l_payload);
        strcat(v_envValues[(*itpub).device - 1].hum, "%");

        break;
      }
      case 3:
      {
      strcat(l_topic, "pres");
        strcpy(v_envValues[(*itpub).device - 1].pres, "");
        strcat(v_envValues[(*itpub).device - 1].pres, l_payload);
//        strcat(v_envValues[(*itpub).device - 1].pres, " hPa");
        break;
      }
      case 4:
      {
              strcat(l_topic,"volt");
        strcpy(v_envValues[(*itpub).device - 1].volt, "");
        strcat(v_envValues[(*itpub).device - 1].volt, l_payload);
//        strcat(v_envValues[(*itpub).device - 1].volt, " V");
        break;
      }
      case 5:
      {
      strcat(l_topic, "co2");
        strcpy(v_envValues[(*itpub).device - 1].co2, "");
        strcat(v_envValues[(*itpub).device - 1].co2, l_payload);
 //       strcat(v_envValues[(*itpub).device - 1].co2, " ppm");
        break;
      }
      }

      // sprintf (buffer, "%d plus %d is %d", a, b, a+b);

      Serial.print(millis());
      Serial.print(":  publish   ");
      Serial.print(l_topic);
      Serial.print("  /  ");
      Serial.print(l_payload);
      Serial.print("  / result: ");
      Serial.println(publishTopic(l_topic, l_payload));
      // !!!!!!!!!!!!!!!!!!!!!checki if publish happend and delete entry from vector
    }

    v_publish.clear();
  }
}