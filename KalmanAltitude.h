//---------------------------------------------------------------------------
#ifndef KalmanAltitudeH
#define KalmanAltitudeH
#include "Globals.h"
#include "matrix.h"

//---------------------------------------------------------------------------
class KalmanAltitude {
public:
   Matrix *A, *H, *Q, *R;
	Matrix *x, *P, *xp, *Pp, *K, *z;
	Matrix *t;
	~KalmanAltitude();
   KalmanAltitude( int numStates, int numMeasurements );
   void filter();
	void init();
private:
	float dt;
};
//---------------------------------------------------------------------------
#endif
