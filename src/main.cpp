

/*
   AC Power Calculations
   Average Power, Apparent Power, Reactive Power, Irms, Vrms, Power Factor,
      Power Factor Angle
   Date: Dec. 11, 2009
         Feb. 20, 2022 - added MQTT, removed LCD support, external button

   Line Current is measured using a current transformer from CR Magnetics.

   CR8410-1000 @ http://www.crmagnetics.com/8400.pdf
   Line voltage measurement via a standard AC step down transformer


   Author: peter c
 */

#include <Arduino.h>
#include <string.h>
#include <TimeLib.h>

// #include <MemoryFree.h>
#include <Streaming.h>
#include <Ethernet.h>

#if defined(MODBUS_IP)

#include <MgsModbus.h> // cchange memory size here

#else // if defined(MODBUS_IP)
#include <ModbusRtu.h>
#endif // if defined(MODBUS_IP)

#include <ACSignal.h>
#include <PowerSignal.h>

#include <DA_AnalogInput.h>
#include <DA_DiscreteInput.h>
#include <DA_NonBlockingDelay.h>

#define DEFAULT_IP_ADDRESS 192, 168, 1, 85
#define DEFAULT_GATEWAY 192, 168, 1, 254
#define DEFAULT_SUBNET_MASK 255, 255, 255, 0
#define DEFAULT_MAC_ADDRESS 0xDE, 0xAD, 0xBE, 0x00, 0x01, 0xFD
IPAddress defaultIP(DEFAULT_IP_ADDRESS);
IPAddress defaultGateway(DEFAULT_GATEWAY);
IPAddress defaultSubnet(DEFAULT_SUBNET_MASK);
byte defaultMAC[] = {DEFAULT_MAC_ADDRESS};

#if defined(MODBUS_IP)
MgsModbus slave;
#define modbusRegisters slave.MbData

void readSetpointsFromMaster();
void refreshModbusRegisters();
void processModbusCommands();

union
{
  unsigned int regsf[2];
  float val;
} bfconvert;

union
{
  unsigned int regsl[2];
  long val;
} blconvert;

#else // MQTT
#include <PubSubClient.h>

#include <ArduinoJson.h>

#define MQTT_PUBLISH_RATE 30000 // ms
#define MAX_MSG_OUT_SZ 250
IPAddress mqtt_server(192, 168, 1, 81);
String clientID = "power001";
const char *hostCommandTopic = "power001";
const char *mqttTopic = "Home/Power/power001/EU";
char mqttMsgOut[MAX_MSG_OUT_SZ];
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);
StaticJsonDocument<100> mqttMsgIn;
void onMQTTMessage(char *topic, byte *payload, unsigned int length);
void onpublishDataTmr();
void reconnect();

#endif

void onPowerCalc();

// #define DEBUGPWR  // DEBUG

#define CURRENT_BLACK_PIN 2 // current 1st 100 AMP input pin (black )
#define CURRENT_RED_PIN 3   // current 1st 100 AMP input pin (red )
#define VOLTAGEPIN 1        // voltage signal input pin

#define DC_OFFSET 2.5 // signal shifted 2.5 above ground

#define Te 2011.0   // current transformer effective turns
                    // ratio from speck sheet
#define CT_VMAX 2.5 // maximum level shifted voltage from CT
                    // to input e.g. 7.2 v rms to 2.5 v-pp
                    // offset

#define CT_VRATED 10.61891 // Max rated RMS voltage * sqrt(2) of CT
                           // from spec sheed
// #define CT_VRATED     7.03239            // Max rated RMS voltage * sqrt(2)
// of CT from spec sheed
#define CT_LOAD_RES 151.0 // current transformer load resistor to
                          // create a voltage drop
// #define CT_LOAD_RES   100.0               // current transformer load
// resistor to create a voltage drop
#define MAX_SAMPLES 2000 // Samples per Calculation

#define PEAK_LINE_VOLTAGE 170        // 120 * sqrt(2)
#define PEAK_VOLTAGE_TRANSFORMER 2.5 // input to A2D max AC value

#define TEMP_CONV 150. / (1.5 * 1023 / 5)

#define BLACK_INDEX 0 // offsets into array for holding power values
#define RED_INDEX 1
#define MAX_PRECISION 10

#define SAMPLING_INTERVAL 50 // milli-seconds

// Reset totalizers at 12:00 midnight every day
#define HOUR_RESET 0
#define MINUTE_RESET 0
#define SECOND_RESET 4

// DA_AnalogInput TI_001 = DA_AnalogInput(5, 0.0, 1023.0);
// RunningMedian TI_001MedianFilter = RunningMedian(5);

DA_NonBlockingDelay powerCalcTmr = DA_NonBlockingDelay(5000, onPowerCalc);
DA_NonBlockingDelay publishDataTmr = DA_NonBlockingDelay(MQTT_PUBLISH_RATE, onpublishDataTmr);

// ACSignal voltageSignal( VOLTAGEPIN, 1, DC_OFFSET );
// ACSignal currentSignalBlack( CURRENT_BLACK_PIN, 1, DC_OFFSET );

ACSignal voltageSignal(VOLTAGEPIN,
                       PEAK_LINE_VOLTAGE / PEAK_VOLTAGE_TRANSFORMER,
                       DC_OFFSET);
ACSignal currentSignalBlack(CURRENT_BLACK_PIN,
                            Te / CT_LOAD_RES * CT_VRATED / CT_VMAX,
                            DC_OFFSET);
ACSignal currentSignalRed(CURRENT_RED_PIN,
                          Te / CT_LOAD_RES * CT_VRATED / CT_VMAX,
                          DC_OFFSET);

PowerSignal panelBlack(voltageSignal, currentSignalBlack);
PowerSignal panelRed(voltageSignal, currentSignalRed);

// PowerSignal panelBlack(voltageSignal, voltageSignal );

time_t gPCtime;

void setup()
{

#if defined(DEBUG)
  Serial.begin(9600);

#endif

#if not defined(MODBUS_IP)
  mqttClient.setBufferSize(MAX_MSG_OUT_SZ);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(onMQTTMessage);

#endif

  Ethernet.begin(defaultMAC, defaultIP, defaultGateway, defaultSubnet);
  delay(1500);
}

void loop()
{

#if defined(MODBUS_IP)
  slave.MbsRun();
  refreshModbusRegisters();
  processModbusCommands();

#else // MQTT

  publishDataTmr.refresh();
  //  if (!mqttClient.connected()) {
  //  reconnect();
  // }
  mqttClient.loop();

#endif

  // resetTotalizers();
  // TI_001.refresh();
  powerCalcTmr.refresh();
}

void onPowerCalc()
{
  //  Serial << "pwerCal" << endl;
  voltageSignal.beginSampling();
  currentSignalBlack.beginSampling();
  currentSignalRed.beginSampling();

  panelBlack.beginSampling();
  panelRed.beginSampling();

  for (int i = 0; i < MAX_SAMPLES; i++)
  {
    voltageSignal.acquireSignalSample();
    currentSignalBlack.acquireSignalSample();
    currentSignalRed.acquireSignalSample();
    panelBlack.updateAccumulator();
    panelRed.updateAccumulator();
  }

  voltageSignal.endSampling();
  currentSignalBlack.endSampling();
  currentSignalRed.endSampling();
  panelBlack.endSampling();
  panelRed.endSampling();
}

void TI_001_Callback(float aValue)
{
  // Serial << " temperature:" << aValue * TEMP_CONV << endl;
  // TI_001MedianFilter.add(aValue * TEMP_CONV);
}

void resetTotalizers()
{

  if ((hour() == HOUR_RESET) && (minute() == MINUTE_RESET) &&
      (second() < SECOND_RESET))
  {
    panelBlack.resetTotalizers();
    panelRed.resetTotalizers();
  }
}

#if defined(MODBUS_IP)
void processModbusCommands()
{

  if (modbusRegisters[32] != 0)
  {
    blconvert.regsl[0] = modbusRegisters[32];
    blconvert.regsl[1] = modbusRegisters[33];

    gPCtime = blconvert.val;
    setTime(gPCtime);
    modbusRegisters[32] = 0;
    modbusRegisters[33] = 0;
  }
}

void refreshModbusRegisters()
{
  modbusRegisters[0] = (int)(voltageSignal.getRMSValue() * 100.0);
  modbusRegisters[1] = (int)(voltageSignal.getCrestFactor() * 100.0);

  modbusRegisters[2] = (int)(currentSignalBlack.getRMSValue() * 100.0);
  modbusRegisters[3] = (int)(currentSignalBlack.getCrestFactor() * 100.0);

  modbusRegisters[4] = (int)(currentSignalRed.getRMSValue() * 100.0);
  modbusRegisters[5] = (int)(currentSignalRed.getCrestFactor() * 100.0);

  // modbusRegisters[ 6 ] = (int) (panelBlack.getDeltaT());
  // modbusRegisters[ 7 ] = (int) (panelRed.getDeltaT());

  modbusRegisters[8] = (int)(TI_001MedianFilter.getMedian() * 10);

  blconvert.val = (long)(panelBlack.computeAveragePower() * 100.0);
  modbusRegisters[9] = blconvert.regsl[0];
  modbusRegisters[10] = blconvert.regsl[1];

  blconvert.val = (long)(panelRed.computeAveragePower() * 100.0);
  modbusRegisters[11] = blconvert.regsl[0];
  modbusRegisters[12] = blconvert.regsl[1];

  blconvert.val = (long)(panelBlack.computeApparentPower() * 100.0);
  modbusRegisters[13] = blconvert.regsl[0];
  modbusRegisters[14] = blconvert.regsl[1];

  blconvert.val = (long)(panelRed.computeApparentPower() * 100.0);
  modbusRegisters[15] = blconvert.regsl[0];
  modbusRegisters[16] = blconvert.regsl[1];

  blconvert.val = (long)(panelBlack.computeReactivePower() * 100.0);
  modbusRegisters[17] = blconvert.regsl[0];
  modbusRegisters[18] = blconvert.regsl[1];

  blconvert.val = (long)(panelRed.computeReactivePower() * 100.0);
  modbusRegisters[19] = blconvert.regsl[0];
  modbusRegisters[20] = blconvert.regsl[1];

  blconvert.val = (long)(panelBlack.getKiloWattsHrs() * 10000.0);
  modbusRegisters[23] = blconvert.regsl[0];
  modbusRegisters[24] = blconvert.regsl[1];

  blconvert.val = (long)(panelRed.getKiloWattsHrs() * 10000.0);
  modbusRegisters[25] = blconvert.regsl[0];
  modbusRegisters[26] = blconvert.regsl[1];

  modbusRegisters[21] = (int)(panelBlack.computePowerFactor() * 100.0);
  modbusRegisters[22] = (int)(panelRed.computePowerFactor() * 100.0);

  //    modbusRegisters[ 8 ] = (int) (panelBlack.computePowerFactor() * 100.0);
}
#else // MQTT
void onMQTTMessage(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

#endif
}

void reconnect()
{

  if (!mqttClient.connected())
  {
#ifdef DEBUG
    Serial << "Attempting MQTT connection..." << endl;
#endif
    // Create a random client ID
    clientID += String(random(0xffff), HEX);
    // Attempt to connect
    if (mqttClient.connect(clientID.c_str()))
    //  if (mqttClient.connect("arduinoClient"))
    {
#ifdef DEBUG
      Serial << "Connected.." << endl;
#endif
      // mqttClient.publish("outTopic", "hello world");
      mqttClient.subscribe(hostCommandTopic);
    }
    else
    {
#ifdef DEBUG
      Serial << "MQTT Connect failed:(" << mqttClient.state() << ")" << endl;
#endif
    }
  }
}

/*
EI-001	Main Voltage RMS
EIB-001	Main Crest Factor
II-001	Total Current A
II-001B	Main Current (Black) A RMS
II-001R	Main Current (Red) A RMS
CIB-001B	Main Current (Black) Crest Factor
CIB-001R	Main Current (Red) Crest Factor
AIA-001	Total Apparent Power VA
AIA-001B	Aparent Power (Black)  VA
AIA-001R	Aparent Power (Red)  VA
FIF-001	Main  Power Factor
FIF-001B	Power Factor (Black)
FIF-001R	Power Factor (Read)
RII-001	Total Reactive Power var
RII-001B	Reactive Power (Black)  var
RII-001R	Reactive Power (Red)  var
PIR-001	Total Rear Power W
PIR-001B	Real Power (Black)  W
PIR-001R	Real Power (Red)  W


*/
void onpublishDataTmr()
{

  if (!mqttClient.connected())
  {
    reconnect();
  }

  float totalReal = panelBlack.computeAveragePower() + panelRed.computeAveragePower();
  float totalCurrent = currentSignalBlack.getRMSValue() + currentSignalRed.getRMSValue();
  float totalApparent = panelBlack.computeApparentPower() + panelRed.computeApparentPower();
  float totalReactive = panelBlack.computeReactivePower() + panelRed.computeReactivePower();
  float linePowerFactor = (totalApparent > 0) ? totalReal / totalApparent : 0;

  // sprintf(mqttMsgOut,
  //         "{\"EI-001\": %.1f, \"EIB-001\":%.1f, \"II-001\":%.1f, \"II-001B\":%.1f,\"II-001R\": %.1f, \"CIB-001B\":%.1f,"
  //         "\"CIB-001R\":%.1f, \"AIA-001\":%.1f, \"AIA-001B\": %.1f,"
  //         "\"AIA-001R\":%.1f, \"FIF-001\":%.2f, \"FIF-001B\":%.2f,\"FIF-001R\":%.2f,"
  //         "\"RII-001\":%.1f, \"RII-001B\":%.1f, \"RII-001R\":%.1f,"
  //         "\"PIR-001\":%.1f, \"PIR-001B\":%.1f, , \"PIR-001R\":%.1f }",
  //         voltageSignal.getRMSValue(), voltageSignal.getCrestFactor(), totalCurrent, currentSignalBlack.getRMSValue(), currentSignalRed.getRMSValue(), currentSignalBlack.getCrestFactor(),
  //         currentSignalRed.getCrestFactor(), totalApparent, panelBlack.computeApparentPower(),
  //         panelRed.computeApparentPower(), linePowerFactor, panelBlack.computePowerFactor(), panelRed.computePowerFactor(),
  //         totalReactive, panelBlack.computeReactivePower(), panelRed.computeReactivePower(),
  //         totalReal, panelBlack.computeAveragePower(), panelRed.computeAveragePower());

  char str_tmp[8];
  char str_tmp2[8];
  char str_tmp3[8];
  char str_tmp4[8];
  char str_tmp5[8];
  char str_tmp6[8];
  char str_tmp7[8];

  // dtostrf(1.1,4,1, str_tmp);
  // dtostrf(2.2,3,2, str_tmp2);
  // dtostrf(3.3,3,1, str_tmp3);
  // dtostrf(4.4,3,1, str_tmp4);
  // dtostrf(5.5,3,1, str_tmp5);
  // dtostrf(6.6,3,2, str_tmp6);

  dtostrf(voltageSignal.getRMSValue(), 4, 1, str_tmp);
  dtostrf(voltageSignal.getCrestFactor(), 3, 2, str_tmp2);
  dtostrf(totalCurrent, 3, 1, str_tmp3);
  dtostrf(currentSignalBlack.getRMSValue(), 3, 1, str_tmp4);
  dtostrf(currentSignalRed.getRMSValue(), 3, 1, str_tmp5);
  dtostrf(currentSignalBlack.getCrestFactor(), 3, 2, str_tmp6);

  sprintf(mqttMsgOut,
          "{\"EI-001\": %s, \"EIB-001\":%s, \"II-001\":%s, \"II-001B\":%s,\"II-001R\": %s, \"CIB-001B\":%s}",
          str_tmp, str_tmp2, str_tmp3, str_tmp4, str_tmp5, str_tmp6);
  mqttClient.publish(mqttTopic, mqttMsgOut);

  dtostrf(currentSignalRed.getCrestFactor(), 3, 2, str_tmp);
  dtostrf(totalApparent, 5, 1, str_tmp2);
  dtostrf(panelBlack.computeApparentPower(), 5, 1, str_tmp3);
  dtostrf(panelRed.computeApparentPower(), 5, 1, str_tmp4);
  dtostrf(linePowerFactor, 1, 3, str_tmp5);
  dtostrf(panelBlack.computePowerFactor(), 1, 3, str_tmp6);
  dtostrf(panelRed.computePowerFactor(), 1, 3, str_tmp7);

  // dtostrf(7.7,4,1, str_tmp);
  // dtostrf(8.8,3,2, str_tmp2);
  // dtostrf(9.9,3,1, str_tmp3);
  // dtostrf(10.1,3,1, str_tmp4);
  // dtostrf(11.1,3,1, str_tmp5);
  // dtostrf(12.2,3,2, str_tmp6);
  // dtostrf(13.3,3,2, str_tmp7);

  sprintf(mqttMsgOut,
          "{\"CIB-001R\":%s, \"AIA-001\":%s, \"AIA-001B\": %s,\"AIA-001R\":%s, \"FIF-001\":%s, \"FIF-001B\":%s,\"FIF-001R\":%s}",
          str_tmp, str_tmp2, str_tmp3, str_tmp4, str_tmp5, str_tmp6, str_tmp7);
  mqttClient.publish(mqttTopic, mqttMsgOut);

  dtostrf(totalReactive, 4, 1, str_tmp);
  dtostrf(panelBlack.computeReactivePower(), 4, 1, str_tmp2);
  dtostrf(panelRed.computeReactivePower(), 4, 1, str_tmp3);
  dtostrf(totalReal, 5, 1, str_tmp4);
  dtostrf(panelBlack.computeAveragePower(), 5, 1, str_tmp5);
  dtostrf(panelRed.computeAveragePower(), 5, 1, str_tmp6);

  //         totalReactive, panelBlack.computeReactivePower(), panelRed.computeReactivePower(),
  //         totalReal, panelBlack.computeAveragePower(), panelRed.computeAveragePower());

  // dtostrf(14.4,4,1, str_tmp);
  // dtostrf(15.5,3,2, str_tmp2);
  // dtostrf(16.6,3,1, str_tmp3);
  // dtostrf(17.7,3,1, str_tmp4);
  // dtostrf(18.8,3,1, str_tmp5);
  // dtostrf(19.9,3,2, str_tmp6);

  sprintf(mqttMsgOut,
          "{\"RII-001\":%s, \"RII-001B\":%s, \"RII-001R\":%s,\"PIR-001\":%s, \"PIR-001B\":%s, \"PIR-001R\":%s }",
          str_tmp, str_tmp2, str_tmp3, str_tmp4, str_tmp5, str_tmp6);
  mqttClient.publish(mqttTopic, mqttMsgOut);

#ifdef DEBUG
  Serial << "MQTT Topic:" << mqttTopic << endl;
  Serial << "Msg:";
  // Serial << "MQTT Message:" << mqttMsgOut << endl;

  Serial.println(mqttMsgOut);
#endif
}

#endif