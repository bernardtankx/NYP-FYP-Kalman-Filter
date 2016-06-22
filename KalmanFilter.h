//---------------------------------------------------------------------------

#ifndef KalmanFilterH
#define KalmanFilterH
#include "KalmanX.h"
#include "KalmanY.h"
#include "KalmanAngular.h"
#include "KalmanAltitude.h"
//---------------------------------------------------------------------------
class KalmanFilter {
public:

	KalmanFilter();
	~KalmanFilter();
	KalmanX *kalmanX;
	KalmanY *kalmanY;
	KalmanAngular *kalmanAngular;
	KalmanAltitude *kalmanAltitude;

	void filterUpdate( float velocityX, float velocityY, float angularVelocity, float positionX, float positionY, float angularPosition, float altitudePosition );
	void getAllVelocity( float& velocityX, float& velocityY );
	void getAngularVelocity( float& angularVelocity );
	void getAllPosition( float& positionX, float& positionY, float& altitudePosition );
	void getAngularPosition( float& angularPosition );
	void positionReset();
};
//---------------------------------------------------------------------------
#endif
