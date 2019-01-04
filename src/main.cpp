

/*
   AC Power Calculations
   Average Power, Apparent Power, Reactive Power, Irms, Vrms, Power Factor,
      Power Factor Angle
   Date: Dec. 11, 2009

   Line Current is measured using a current transformer from CR Magnetics.

   CR8410-1000 @ http://www.crmagnetics.com/8400.pdf
   Line voltage measurement via a standard AC step down transformer


   Author: peter c
 */

#include <Arduino.h>
#include <string.h>
#include <TimeLib.h>
#include <LCD4Bit.h>

#include <RunningMedian.h> // https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningMedian
// #include <MemoryFree.h>
#include <Streaming.h>

#if defined(MODBUS_IP)

#include <Ethernet.h>
  #include <MgsModbus.h> // cchange memory size here

#else // if defined(MODBUS_IP)
  #include <ModbusRtu.h>
#endif // if defined(MODBUS_IP)

#include <ACSignal.h>
#include <PowerSignal.h>

#include <DA_AnalogInput.h>
#include <DA_DiscreteInput.h>
#include <DA_NonBlockingDelay.h>




#if defined(MODBUS_IP)
MgsModbus slave;
  #define modbusRegisters slave.MbData
  #define DEFAULT_IP_ADDRESS 192, 168, 1, 85
  #define DEFAULT_GATEWAY 192, 168, 1, 254
  #define DEFAULT_SUBNET_MASK 255, 255, 255, 0
  #define DEFAULT_MAC_ADDRESS 0xDE, 0xAD, 0xBE, 0x00, 0x01, 0xFD
IPAddress defaultIP(DEFAULT_IP_ADDRESS);
IPAddress defaultGateway(DEFAULT_GATEWAY);
IPAddress defaultSubnet(DEFAULT_SUBNET_MASK);
byte defaultMAC[] = { DEFAULT_MAC_ADDRESS };
#else // if defined(MODBUS_IP)
#define MODBUS_REG_COUNT 50
#define MB_SLAVE_ID                     1
#define MB_SERIAL_PORT                  0
#define MB_SERIAL_BAUD                  19200
#define MB_MAX_REGISTERS 35 // maximum Modbus registers
uint16_t modbusRegisters[MODBUS_REG_COUNT];
Modbus   slave(MB_SLAVE_ID, MB_SERIAL_PORT);
#endif // if defined(MODBUS_IP)


// forward declarations
float readFilteredTemperature(int   aPin,
                              float aFilterCoefficient,
                              float aFilteredValue);
float readTemperature(int aPin,
                      int aSamples);
void  resetTotalizers();
void  readSetpointsFromMaster();
void  refreshModbusRegisters();
void  processModbusCommands();
void  displayResults();
void  displayVals(int   aRow,
                  char *aLabel1,
                  float aValue1,
                  char *aLabel2,
                  float aValue2,
                  int   aPrecision);

void TI_001_Callback(float aValue);
void LS_001_Callback(bool aValue,
                     int  aPin);

void onPowerCalc();

// #define DEBUGPWR  // DEBUG

#define LCD_DISPLAY_WIDTH  15
#define LCD_DEGREE_CHAR 0xF2


#define CURRENT_BLACK_PIN 2            // current 1st 100 AMP input pin (black )
#define CURRENT_RED_PIN   3            // current 1st 100 AMP input pin (red )
#define VOLTAGEPIN 1                   // voltage signal input pin


#define DC_OFFSET 2.5                  // signal shifted 2.5 above ground

#define Te            2011.0           // current transformer effective turns
                                       // ratio from speck sheet
#define CT_VMAX       2.5              // maximum level shifted voltage from CT
                                       // to input e.g. 7.2 v rms to 2.5 v-pp
                                       // offset

#define CT_VRATED     10.61891         // Max rated RMS voltage * sqrt(2) of CT
                                       // from spec sheed
// #define CT_VRATED     7.03239            // Max rated RMS voltage * sqrt(2)
// of CT from spec sheed
#define CT_LOAD_RES   151.0            // current transformer load resistor to
                                       // create a voltage drop
// #define CT_LOAD_RES   100.0               // current transformer load
// resistor to create a voltage drop
#define MAX_SAMPLES   2000             // Samples per Calculation

#define PEAK_LINE_VOLTAGE   170        // 120 * sqrt(2)
#define PEAK_VOLTAGE_TRANSFORMER   2.5 // input to A2D max AC value

#define TEMP_CONV  150. / (1.5 * 1023 / 5)

#define BLACK_INDEX 0 // offsets into array for holding power values
#define RED_INDEX   1
#define MAX_PRECISION 10


#define DISPLAY_RMS               1
#define DISPLAY_REAL_POWER        2 // Real and Apparent
#define DISPLAY_APPARENT__POWER   3
#define DISPLAY_POWER_FACTORS     4 // Power Factor
#define DISPLAY_PFA               5 // Power Factor Angle
#define DISPLAY_CREST_FACTOR      6 //
#define DISPLAY_LINE_SPECS        7 // line voltage and crest factor
#define DISPLAY_C02               8 // Killowatt hour and C02
#define DISPLAY_TEMPERATURE       9
#define DISPLAY_TIME              10
#define MAX_DISPLAY_MODE          10


#define SAMPLING_INTERVAL         50 // milli-seconds


// Reset totalizers at 12:00 midnight every day
#define HOUR_RESET   0
#define MINUTE_RESET 0
#define SECOND_RESET 4


union
{
  unsigned int regsf[2];
  float        val;
}
bfconvert;

union
{
  unsigned int regsl[2];
  long         val;
}
blconvert;


DA_AnalogInput TI_001             = DA_AnalogInput(5, 0.0, 1023.0);
RunningMedian  TI_001MedianFilter = RunningMedian(5);

DA_DiscreteInput LS_001 = DA_DiscreteInput(4,
                                           DA_DiscreteInput::FallingEdgeDetect,
                                           false);
DA_NonBlockingDelay powerCalcTmr = DA_NonBlockingDelay(5000, onPowerCalc);

// create object to control an LCD.
// number of lines in display=1

LCD4Bit lcd = LCD4Bit(2);


// ACSignal voltageSignal( VOLTAGEPIN, 1, DC_OFFSET );
// ACSignal currentSignalBlack( CURRENT_BLACK_PIN, 1, DC_OFFSET );

ACSignal voltageSignal(VOLTAGEPIN,
                       PEAK_LINE_VOLTAGE / PEAK_VOLTAGE_TRANSFORMER,
                       DC_OFFSET);
ACSignal currentSignalBlack(CURRENT_BLACK_PIN,
                            Te / CT_LOAD_RES *CT_VRATED / CT_VMAX,
                            DC_OFFSET);
ACSignal currentSignalRed(CURRENT_RED_PIN,
                          Te / CT_LOAD_RES *CT_VRATED / CT_VMAX,
                          DC_OFFSET);

PowerSignal panelBlack(voltageSignal, currentSignalBlack);
PowerSignal panelRed(voltageSignal, currentSignalRed);


// PowerSignal panelBlack(voltageSignal, voltageSignal );

int g_displayMode =  MAX_DISPLAY_MODE;


time_t gPCtime;


// Simplistic Float to Alhpa used for displaying floats on the LCD
// decimal is good for 1 to 5. no error checks. garbage in - garbage out.
// problem: number like 1.01 => 1.009 need to look into that later.
char* poor_ftoa(char *a_buffer, float a_number, int a_decimals)
{
  char  l_buffer[MAX_PRECISION + 1];
  int   l_multiplier;
  float x;


  l_multiplier = 10;
  x            = a_number - (int)a_number;

  if (x < 0) x *= -1;

  for (int i = 0; i < a_decimals; i++)
  {
    x *= l_multiplier;

    l_buffer[i] =  '0' + (int)(x);

    x -= (int)x;
  }
  l_buffer[a_decimals] = '\0';

  if (a_decimals > 0) sprintf(a_buffer, "%0d.%s", (int)a_number, l_buffer);
  else sprintf(a_buffer, "%0d", (int)a_number);

  return a_buffer;
}

void setup()
{



  lcd.init();


  TI_001.setOnPollCallBack(TI_001_Callback);
  LS_001.setOnEdgeEvent(LS_001_Callback);
  LS_001.setPollingInterval(250);  // ms
  TI_001.setPollingInterval(2500); // ms


    #if defined(MODBUS_IP)

  Ethernet.begin(defaultMAC, defaultIP, defaultGateway, defaultSubnet);

  #else // if defined(MODBUS_IP)

  slave.begin(MB_SERIAL_BAUD);
  #endif // if defined(MODBUS_IP)

  Serial.begin( 9600);
  Serial << "hello from " << Ethernet.localIP() << endl;



  // Serial.println("Start");
}

void loop()
{




 #if defined(MODBUS_IP)
  slave.MbsRun();

#else // if defined(MODBUS_IP)

  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  displayResults();
  LS_001.refresh();

#endif // if defined(MODBUS_IP)

refreshModbusRegisters();
processModbusCommands();

//resetTotalizers();
TI_001.refresh();
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

void LS_001_Callback(bool aValue, int aPin)
{
  // Serial << " switch " << endl;
  g_displayMode++;

  if (g_displayMode > MAX_DISPLAY_MODE) g_displayMode = DISPLAY_RMS;
}

void TI_001_Callback(float aValue)
{
  // Serial << " temperature:" << aValue * TEMP_CONV << endl;
  TI_001MedianFilter.add(aValue * TEMP_CONV);
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

void processModbusCommands()
{
  if (modbusRegisters[30] != 0)
  {
    g_displayMode       = modbusRegisters[30];
    modbusRegisters[30] = 0;
  }


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


  blconvert.val       = (long)(panelBlack.computeAveragePower() * 100.0);
  modbusRegisters[9]  = blconvert.regsl[0];
  modbusRegisters[10] = blconvert.regsl[1];

  blconvert.val       = (long)(panelRed.computeAveragePower() * 100.0);
  modbusRegisters[11] = blconvert.regsl[0];
  modbusRegisters[12] = blconvert.regsl[1];

  blconvert.val       = (long)(panelBlack.computeApparentPower() * 100.0);
  modbusRegisters[13] = blconvert.regsl[0];
  modbusRegisters[14] = blconvert.regsl[1];

  blconvert.val       = (long)(panelRed.computeApparentPower() * 100.0);
  modbusRegisters[15] = blconvert.regsl[0];
  modbusRegisters[16] = blconvert.regsl[1];

  blconvert.val       = (long)(panelBlack.computeReactivePower() * 100.0);
  modbusRegisters[17] = blconvert.regsl[0];
  modbusRegisters[18] = blconvert.regsl[1];

  blconvert.val       = (long)(panelRed.computeReactivePower() * 100.0);
  modbusRegisters[19] = blconvert.regsl[0];
  modbusRegisters[20] = blconvert.regsl[1];

  blconvert.val       = (long)(panelBlack.getKiloWattsHrs() * 10000.0);
  modbusRegisters[23] = blconvert.regsl[0];
  modbusRegisters[24] = blconvert.regsl[1];

  blconvert.val       = (long)(panelRed.getKiloWattsHrs() * 10000.0);
  modbusRegisters[25] = blconvert.regsl[0];
  modbusRegisters[26] = blconvert.regsl[1];

  modbusRegisters[21] = (int)(panelBlack.computePowerFactor() * 100.0);
  modbusRegisters[22] = (int)(panelRed.computePowerFactor() * 100.0);


  //    modbusRegisters[ 8 ] = (int) (panelBlack.computePowerFactor() * 100.0);
}

/*
   char *timestamptoString(char* aStringBuffer )
   {
   sprintf( aStringBuffer, " %d/%d/%d %d:%02d:%02d", day(), month(), year(),
      hour(), minute(), second() );
   return( aStringBuffer );
   }
 */
void displayResults()
{
  char aBuffer[16];

  switch (g_displayMode)

  // switch(  g_displayMode  )
  {
  case DISPLAY_RMS:
    lcd.cursorTo(1, 0);
    lcd.printIn("Current (Amps)  ");

    //  displayVals( "I",  currentSignalBlack.getRMSValue() , "A",
    // "V",voltageSignal.getRMSValue(), "V");
    displayVals(2,
                (char *)"B>",
                currentSignalBlack.getRMSValue(),
                (char *)"R>",
                currentSignalRed.getRMSValue(),
                1);

    //       displayVals( 2, "B>",  1.2,   "R>",  3.4, 1 );
    break;

  case DISPLAY_REAL_POWER:

    // lcd.clear();
    lcd.cursorTo(1, 0);
    lcd.printIn((char *)"Power (Watts)  ");
    displayVals(2,
                "B>",
                panelBlack.computeAveragePower(),
                "R>",
                panelRed.computeAveragePower(),
                0);
    break;

  case DISPLAY_APPARENT__POWER:
    lcd.cursorTo(1, 0);
    lcd.printIn("Apparent PWR(VA)");
    displayVals(2,
                "B>",
                panelBlack.computeApparentPower(),
                "R>",
                panelRed.computeApparentPower(),
                0);
    break;

  case DISPLAY_POWER_FACTORS:
    lcd.cursorTo(1, 0);
    lcd.printIn("PWR Factor      ");
    displayVals(2,
                "B>",
                panelBlack.computePowerFactor(),
                "R>",
                panelRed.computePowerFactor(),
                1);
    break;

  case DISPLAY_PFA:
    lcd.cursorTo(1, 0);
    lcd.printIn("PFA (Degrees)   ");
    displayVals(2,
                "B>",
                panelBlack.computePowerFactorAngle(PS_DEGREES),
                "R>",
                panelRed.computePowerFactorAngle(PS_DEGREES),
                1);
    break;

  case DISPLAY_CREST_FACTOR:
    lcd.cursorTo(1, 0);
    lcd.printIn("Crest Factor    ");
    displayVals(2,
                "B>",
                currentSignalBlack.getCrestFactor(),
                "R>",
                currentSignalRed.getCrestFactor(),
                1);
    break;

  case DISPLAY_LINE_SPECS:
    lcd.cursorTo(1, 0);
    lcd.printIn((char *)"Line Specs      ");

    displayVals(2,
                "V>",
                voltageSignal.getRMSValue(),
                "CF>",
                voltageSignal.getCrestFactor(),
                1);
    break;

  case DISPLAY_C02:
    lcd.cursorTo(1, 0);

    if (panelBlack.getKiloWattsHrs() < 1.0)
    {
      lcd.printIn((char *)"W*hr and CO2-g  ");
      displayVals(2,
                  ">",
                  panelBlack.getKiloWattsHrs() * 1000,
                  ">",
                  panelBlack.getCO2Emissions() * 373.2417,
                  1); // covert to grams
    }
    else
    {
      lcd.printIn("kW*hr & CO2-lbs");
      displayVals(2,
                  ">",
                  panelBlack.getKiloWattsHrs(),
                  ">",
                  panelBlack.getCO2Emissions(),
                  1);
    }
    break;

  case DISPLAY_TIME:


    lcd.cursorTo(1, 0);

    sprintf(aBuffer, "Date: %d/%d/%d", day(), month(), year());
    lcd.printIn(aBuffer);
    lcd.cursorTo(2, 0);
    sprintf(aBuffer, "Time: %d:%02d:%02d ", hour(), minute(), second());
    lcd.printIn(aBuffer);
    break;

  case DISPLAY_TEMPERATURE:


    lcd.cursorTo(1, 0);
    lcd.printIn((char *)"Temperature     ");
    displayVals(2, (char const *)"a>", TI_001MedianFilter.getAverage(),
                (char const *)"x>", 0, 1);


    break;

  default:
    g_displayMode =  DISPLAY_LINE_SPECS;
    break;
  }
}

void displayVals(int   aRow,
                 char *aLabel1,
                 float aValue1,
                 char *aLabel2,
                 float aValue2,
                 int   aPrecision)
{
  char l_buffer[LCD_DISPLAY_WIDTH + 1];

  char l_value[15];
  char l_value2[15];
  int  l_len;


  // fill display buffer with blanks
  //  memset (l_buffer,'P',LCD_DISPLAY_WIDTH);


  // l_buffer[ LCD_DISPLAY_WIDTH + 1  ] = '\0';
  //     Serial.println( l_buffer );
  strcpy(l_buffer, "               ");


  // lcd.clear();
  lcd.cursorTo(aRow, 0);

  poor_ftoa(l_value,  aValue1, aPrecision);
  poor_ftoa(l_value2, aValue2, aPrecision);
  sprintf(l_buffer, "%s%s %s%s", aLabel1, l_value, aLabel2, l_value2);

  //  ensure blanks fill up to right of display
  l_len = strlen(l_buffer);

  if (l_len < LCD_DISPLAY_WIDTH) l_buffer[l_len] = ' ';


  lcd.printIn(l_buffer);
}
