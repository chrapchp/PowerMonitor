/*
  PowerSignal.cpp - Library doing basic signal calculations
  Created by Peter Chrapchynski, December 11, 2009

*/

//#include "WProgram.h"
#include <Arduino.h>  // was WProgram.h, changed to Arduino.h in 1.0
#include "ACSignal.h"
#include "PowerSignal.h"

#define PS_TRUE  1
#define PS_FALSE 0


PowerSignal::PowerSignal(ACSignal& aVoltageSignal, ACSignal& aCurrentSignal):mVoltageSignal( aVoltageSignal ), mCurrentSignal( aCurrentSignal )
{
   mPercentFromCoal = PERCENT_FROM_COAL;
   mLbsCO2PerKWH = LBS_OF_CO2_PER_KWH;
   mAvgCostPerKWH = AVERAGE_COST_PER_KWH;
   mSignalSampled = PS_FALSE;
   mAccumulatedkWHr = 0;
   mDeltaT = 0;
}

void  PowerSignal::resetTotalizers()
{
	mAccumulatedkWHr = 0.0;
}

void  PowerSignal::setPercentFromCoal( float aPercentFromCoal )
{

	mPercentFromCoal = aPercentFromCoal;

}


void PowerSignal::setLbsOfCO2PerKWH( float aLbsOfCO2PerKWH )
{
    mLbsCO2PerKWH = aLbsOfCO2PerKWH;

}


void PowerSignal::setAverageCostPerKWH( float aAverageCostPerKWH )
{
	 mAvgCostPerKWH = aAverageCostPerKWH;
}



float PowerSignal::getPercentFromCoal()
{
	return( mPercentFromCoal /100.0 );
}


float PowerSignal::getLbsOfCO2PerKWH()
{
	return( mLbsCO2PerKWH );
}

float PowerSignal::getAverageCostPerKWH()
{
	return( mAvgCostPerKWH /100.0 );
}



float PowerSignal::computeApparentPower()
{
  return ( mVoltageSignal.getRMSValue() * mCurrentSignal.getRMSValue() );
}


float PowerSignal::computePowerFactor()
{
	float l_powerFactor = 1.0;
	float l_apparentPower = computeApparentPower();

	if( l_apparentPower > 0.01 )
	  l_powerFactor =  computeAveragePower() / l_apparentPower;

    return( l_powerFactor );

}


float PowerSignal::computePowerFactorAngle( int aMode )
{
	float l_PowerFactorAngle = acos( computePowerFactor() );
	if( aMode == PS_DEGREES )
	  l_PowerFactorAngle *= 180 / PI;

	return( l_PowerFactorAngle );
}


float PowerSignal::computeReactivePower()
{
	return( computeApparentPower()  * sin( computePowerFactorAngle( PS_RADIANS ) ) );
}


void PowerSignal::beginSampling()
{

   mSumVoltageTimesCurrent = 0;

}


void PowerSignal::endSampling()
{
	unsigned long l_t;

	if( mSignalSampled == PS_TRUE )
	{

	  l_t = millis();

	  mDeltaT = l_t -  mPreviousSampleTime;
	  mAccumulatedkWHr += computeAveragePower() / 3600.0 /1000.0 * mDeltaT / 1000.0 ;


	  mPreviousSampleTime = l_t;

    }
	else
	{
	   mSignalSampled = PS_TRUE;
	   mPreviousSampleTime = millis();
   }
}

void PowerSignal::updateAccumulator()
{
  mSumVoltageTimesCurrent += mVoltageSignal.getInstantaneousValue() * mCurrentSignal.getInstantaneousValue();
}




double PowerSignal::computeAveragePower()
{



	return( mSumVoltageTimesCurrent / mVoltageSignal.getSampleCount() );
}

unsigned long PowerSignal::getDeltaT()
{
	return( mDeltaT );
}

double  PowerSignal::getKiloWattsHrs()
{
//	return(1001);
  return( mAccumulatedkWHr );



}

double PowerSignal::getCostOfPower()
{
	return( getKiloWattsHrs() * getAverageCostPerKWH() );
}



double  PowerSignal::getCO2Emissions()
{
  return( getKiloWattsHrs() * getPercentFromCoal() * getLbsOfCO2PerKWH() );
}


/*
char* PowerSignal::toString( char* aBuffer )
{

   char l_buffer[ 8 ];
   char l_buffer2[ 8 ];
   char l_buffer3[ 8 ];
   char l_buffer4[ 8 ];
   char l_buffer5[ 8 ];


   sprintf( aBuffer, "%s:%s,%s:%s,%s:%s,%s:%s,%s:%s,%s:%d",
     "AveragePower", poor_atof(l_buffer,computeAveragePower(), 2),
     "ApparentPower", poor_atof(l_buffer2,computeApparentPower(),2 ),
     "ReactivePower", poor_atof(l_buffer3,computeReactivePower(),2 ),
     "PowerFactor", poor_atof(l_buffer4,computePowerFactor(),2 ),
     "PowerFactorAngle", poor_atof(l_buffer5,computePowerFactorAngle( PS_DEGREES),2 ),
     "DeltaT", getDeltaT()
     );



   return( aBuffer );
}



char* PowerSignal::toDbgString( char* aBuffer )
{

   char l_buffer[ 8 ] ;
   char l_buffer2[ 8 ];
   char l_buffer3[ 8 ];
   char l_buffer4[ 8 ];
   char l_buffer5[ 8 ];
   char l_buffer6[ 12 ];
   char l_buffer7[ 12 ];
   char l_buffer8[ 12 ];


      sprintf( aBuffer, "%s=%s,%s=%s,%s=%s,%s=%s,%s=%s,%s=%s,%s=%s,%s=%s,,%s=%d",
        "AveragePower", poor_atof(l_buffer,computeAveragePower(), 2),
        "ApparentPower", poor_atof(l_buffer2,computeApparentPower(),2 ),
        "ReactivePower", poor_atof(l_buffer3,computeReactivePower(),2 ),
        "PowerFactor", poor_atof(l_buffer4,computePowerFactor(),2 ),
        "PowerFactorAngle", poor_atof(l_buffer5,computePowerFactorAngle( PS_DEGREES),2 ),
        "kWHr", poor_atof(l_buffer6, getKiloWattsHrs(),5 ) ,
        "CO2", poor_atof(l_buffer7,getCO2Emissions(),5) ,
        "Cost", poor_atof(l_buffer8,getCostOfPower(),5 ),
        "DeltaT", getDeltaT()
     );


   return( aBuffer );
}

*/
// Simplistic Float to Alhpa used for displaying floats on the LCD
// decimal is good for 1 to 5. no error checks. garbage in - garbage out.
// problem: number like 1.01 => 1.009 need to look into that later.

/*
char* PowerSignal::poor_atof( char *a_buffer, float a_number, int a_decimals )
{
  int l_decPart;
  int l_precision = a_decimals;
  char l_buffer[ 12 ];
  int l_multiplier;
  float x;



  l_multiplier = 10;
  x=a_number - (int)a_number;
  if( x < 0 )
    x *= -1;
  for( int i=0; i<a_decimals; i++ )
  {
    x  *= l_multiplier;
    l_buffer [ i ] =  '0' + (int) ( x );

    x -= (int) x;
  }

  l_buffer[ a_decimals  ] = '\0';
  if( a_decimals > 0 )
    sprintf(a_buffer,"%0d.%s", (int)a_number, l_buffer);
  else
    sprintf(a_buffer,"%0d", (int)a_number);

  return( a_buffer );

}
*/