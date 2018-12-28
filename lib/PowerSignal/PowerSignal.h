/*
  PowerSignal.h - A simplistic way to represent a power signal for AC power monitoring
  Author: peter c
*/
#ifndef PowerSignal_h
#define PowerSignal_h

//#include "WProgram.h"
#include "ACSignal.h"
#include <Arduino.h>  // was WProgram.h, changed to Arduino.h in 1.0

#define PS_DEGREES  1
#define PS_RADIANS 2

#define PERCENT_FROM_COAL      48.8   // % power in jurisdiction from coal Alberta 48.8%
#define LBS_OF_CO2_PER_KWH     2.0    // 2 lbs of CO2/kWh
#define AVERAGE_COST_PER_KWH   8.315  // 8.315 cents per kWh


class PowerSignal
{
  public:
    PowerSignal(ACSignal& aVoltageSignal, ACSignal& aCurrentSignal);


    void setPercentFromCoal( float aPercentFromCoal );
    void setLbsOfCO2PerKWH( float aLbsOfCO2PerKWH );
    void setAverageCostPerKWH( float aAverageCostPerKWH );


    float getPercentFromCoal();
    float getLbsOfCO2PerKWH();
    float getAverageCostPerKWH();

    float computeApparentPower();
    double computeAveragePower();
    float computePowerFactor();
    float computePowerFactorAngle( int aMode );
    float computeReactivePower();
    void beginSampling();
    void updateAccumulator();
    void endSampling();


    double getKiloWattsHrs();
    double getCostOfPower();
    double getCO2Emissions();
    void resetTotalizers();

    unsigned long getDeltaT();  // time since between updates in ms
   // char* toString( char *aBuffer );
   // char* toDbgString( char* aBuffer );



  private:
    ACSignal& mVoltageSignal;
    ACSignal& mCurrentSignal;
    double mSumVoltageTimesCurrent;
    unsigned long mPreviousSampleTime;
    short int mSignalSampled;

    unsigned long mDeltaT;
    double mAccumulatedkWHr;

    float mPercentFromCoal;
    float mLbsCO2PerKWH;
    float mAvgCostPerKWH;
    //char  ff_mtoStringBuffer[100];


    //char* poor_atof(char *a_buffer, float a_number, int a_decimals);
};

#endif
