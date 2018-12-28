/*
  ACSignal.h - A simplistic way to represent a AC signal for AC AC monitoring
  Author: peter c
*/
#ifndef ACSignal_h
#define ACSignal_h

//#include "WProgram.h"
#include <Arduino.h>  // was WProgram.h, changed to Arduino.h in 1.0

#define PS_AD_RESOLUTION 1024.0
#define PS_VOLTS_PER_STEP   0.0048887               // 10 bit A2D reference voltage not 5 but 4.82 => 4.82/1024
#define PS_DEFAULT_DC_OFFSET 2.5                   // AC signal wil be likely 2.5 +/- 2.5
#define PS_PWR_CALC_ERR     -1;
#define PS_SCALING_DEFAULT 1
#define PS_DEFAULT_PORT 1



class ACSignal
{
  public:
    ACSignal();
    ACSignal(int aPin);

    ACSignal(int aPin, float aScalingValue);
    ACSignal(int aPin, float aScalingValue, float aDCOffset);
    void beginSampling();
    void endSampling();
    void setReferenceVoltage( int aReferenceVoltage );
    void setCalibratedCoefficients( float aSlope, float aIntercept );
    float acquireSignalSample();

    void setDCOffset( float aDCOffset );         // AC signal is offset by DC value. typically half-way markt of 0-5 volts
    void setScalingValue( float aScalingValye ); // this is the scaling value used to convert voltage to desired units

    float getDCOffset();
    float getInstantaneousValue();               // return the value of last read sample in scaled units. e.g. amps
    float getMaxValue();
    float getMinValue();


    float getRMSValue();                         // return RMS value of signal
    float getScalingValue();                     // this is the scaling value used to convert voltage to desired units
    float getCrestFactor();                      // compute crest factor ration of peak to RMS
    float getCurrentVoltage();                   // return current voltage read from input (0-5 v)


    int getSampleCount();                        // return numnre of samples taken since beginSample();
    int getCurrentSampleValue();                 // return current sample 0-1023


  //  char* toString( char* aBuffer );
  //  char* toDbgString( char* aBuffer );
private:
    void init( int aPin, float aScalingValue, float aDCOffset );
   // char* poor_atof(char *a_buffer, float a_number, int a_decimals);

  protected:
    int   mPin;
    float mMax;
    float mMin;
    float mAverage;
    float mCurSample;
    int   mcurValue;
    float mScalingValue;
    int   mSampleCount;
    float mSumVoltageRaw;
    float mRealVoltage;
    float mSumVoltageSquared;
    float mCurVoltage;
    float mDCOffset;
    float mVoltsPerStep;
    float mCalibratedSlope;
    float mCalibratedIntercept;
   // char  mtoStringBuffer[300];

};

#endif
