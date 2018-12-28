/*
  ACSignal.cpp - Library doing basic signal calculations
  Created by Peter C, December 11, 2009

*/

//#include "WProgram.h"
#include <Arduino.h>  // was WProgram.h, changed to Arduino.h in 1.0
#include "ACSignal.h"

ACSignal::ACSignal()
{
	init(  PS_DEFAULT_PORT,  PS_SCALING_DEFAULT,  PS_DEFAULT_DC_OFFSET );

}

ACSignal::ACSignal(int aPin)
{
  init(  aPin,  PS_SCALING_DEFAULT,  PS_DEFAULT_DC_OFFSET );


}

ACSignal::ACSignal(int aPin, float aScalingValue)
{
	init(  aPin,  aScalingValue,  PS_DEFAULT_DC_OFFSET );

}

ACSignal::ACSignal(int aPin, float aScalingValue, float aDCOffset)
{
	init(  aPin,  aScalingValue,  aDCOffset );
}

float ACSignal::getRMSValue()
{
  float lRMSValue = PS_PWR_CALC_ERR;
  if( mSampleCount > 0 )
     lRMSValue = sqrt( mSumVoltageSquared / mSampleCount );
  return( lRMSValue );
}

void ACSignal::init( int aPin, float aScalingValue, float aDCOffset )
{

      mPin = aPin;
	  pinMode( mPin, INPUT);
      mScalingValue = aScalingValue;
      mDCOffset = aDCOffset;
      mMin = 0;
      mMax = 0;
      mSampleCount = 0;
      mSumVoltageRaw = 0;
      mSumVoltageSquared =0;
      mVoltsPerStep = PS_VOLTS_PER_STEP;
      mCalibratedSlope = 1.0;
      mCalibratedIntercept = 0.0;
}


int ACSignal::getCurrentSampleValue()
{
	return( mCurSample );
}

void ACSignal::setReferenceVoltage( int aReferenceVoltage )
{
	mVoltsPerStep = aReferenceVoltage / PS_AD_RESOLUTION;
}


void ACSignal::setCalibratedCoefficients( float aSlope, float aIntercept )
{
	mCalibratedSlope = aSlope;
	mCalibratedIntercept = aIntercept;

}

float ACSignal::acquireSignalSample()
{
   float lVal;

   mCurSample = analogRead( mPin )  ;
   mCurVoltage = mCurSample * PS_VOLTS_PER_STEP;
   mSampleCount++;
   mSumVoltageRaw += mCurVoltage;
   mRealVoltage = mCurVoltage - mDCOffset;  // remove DC offset from circuit voltage divider
   lVal = getInstantaneousValue();
   mSumVoltageSquared +=  lVal * lVal;

   if( lVal > mMax )
     mMax = lVal;

   if( lVal < mMin )
      mMin = lVal;



    return( lVal );

}

float  ACSignal::getCrestFactor()
{
	float lRMSValue = getRMSValue();
	float lRetVal = 0.;

	if( lRMSValue > .05 )
	   lRetVal = getMaxValue()/getRMSValue();
    return( lRetVal );
}




float ACSignal::getDCOffset()
{
	return( mDCOffset );

}


void ACSignal::setDCOffset( float aDCOffset )
{
	mDCOffset = aDCOffset;

}

float ACSignal::getInstantaneousValue()
{
	return( mRealVoltage * mScalingValue * mCalibratedSlope + mCalibratedIntercept);
}

float ACSignal::getCurrentVoltage()
{
	return( mCurVoltage );
}

void ACSignal::setScalingValue( float aScalingValue )
{
	   mScalingValue = aScalingValue;
}

float ACSignal::getMaxValue()
{
	return( mMax );
}

float ACSignal::getMinValue()
{
	return( mMin );
}



int ACSignal::getSampleCount()
{
	return( mSampleCount );
}

float  ACSignal::getScalingValue()
{
	return( mScalingValue );
}

void ACSignal::beginSampling()
{
	mSampleCount = 0;
    mSumVoltageRaw =0.0;
	mSumVoltageSquared =0.0;;
	mMax =0;
	mMin = 0;


}


void ACSignal::endSampling()
{
	if( mSampleCount > 0 )
       mDCOffset = mSumVoltageRaw / mSampleCount;
}

