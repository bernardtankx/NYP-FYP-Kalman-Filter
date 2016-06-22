#pragma hdrstop
#include "KalmanFilter.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
KalmanFilter::KalmanFilter()
{
	kalmanAngular = new KalmanAngular( 3 , 2 );
	kalmanAngular->init();
	kalmanX = new KalmanX( 3 , 2 );
	kalmanX->init();
	kalmanY = new KalmanY( 3 , 2 );
	kalmanY->init();
	kalmanAltitude = new KalmanAltitude( 3, 1 );
	kalmanAltitude->init();
}
//---------------------------------------------------------------------------
KalmanFilter::~KalmanFilter()
{
}
//---------------------------------------------------------------------------
void KalmanFilter::filterUpdate( float velocityX, float velocityY, float angularVelocity, float positionX, float positionY, float angularPosition, float altitudePosition )
{
	kalmanX->z->val[ 0 ][ 0 ] = positionX;
	kalmanX->z->val[ 1 ][ 0 ] = velocityX;
	kalmanX->filter();

	kalmanY->z->val[ 0 ][ 0 ] = positionY;
	kalmanY->z->val[ 1 ][ 0 ] = velocityY;
	kalmanY->filter();

	kalmanAngular->z->val[ 0 ][ 0 ] = angularPosition;
	kalmanAngular->z->val[ 1 ][ 0 ] = angularVelocity;
	kalmanAngular->filter();

	kalmanAltitude->z->val[ 0 ][ 0 ] = altitudePosition;
	kalmanAltitude->filter();
}
//---------------------------------------------------------------------------
void KalmanFilter::getAllVelocity( float& velocityX, float& velocityY )
{
	velocityX = kalmanX->x->val[ 1 ][ 0 ];
	velocityY = kalmanY->x->val[ 1 ][ 0 ];
}
//---------------------------------------------------------------------------
void KalmanFilter::getAngularVelocity( float& angularVelocity )
{
	angularVelocity = kalmanAngular->x->val[ 1 ][ 0 ];
}
//---------------------------------------------------------------------------
void KalmanFilter::getAllPosition( float& positionX, float& positionY, float& altitudePosition )
{
	positionX = kalmanX->x->val[ 0 ][ 0 ];
	positionY = kalmanY->x->val[ 0 ][ 0 ];
	altitudePosition = kalmanAltitude->x->val[ 0 ][ 0 ];
}
//---------------------------------------------------------------------------
void KalmanFilter::getAngularPosition( float& angularPosition )
{
	angularPosition = kalmanAngular->x->val[ 0 ][ 0 ];
}
//---------------------------------------------------------------------------
void KalmanFilter::positionReset()
{
	kalmanX->x->val[ 0 ][ 0 ] = 0.0;
	kalmanX->x->val[ 1 ][ 0 ] = 0.0;
	kalmanX->x->val[ 2 ][ 0 ] = 0.0;

	kalmanY->x->val[ 0 ][ 0 ] = 0.0;
	kalmanY->x->val[ 1 ][ 0 ] = 0.0;
	kalmanY->x->val[ 2 ][ 0 ] = 0.0;
}
//---------------------------------------------------------------------------
