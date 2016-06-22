//---------------------------------------------------------------------------
#ifndef KalmanAngularH
#define KalmanAngularH
#include "Globals.h"
#include "matrix.h"

//---------------------------------------------------------------------------
class KalmanAngular {
public:
	Matrix *A, *H, *Q, *R;
	Matrix *x, *P, *xp, *Pp, *K, *z;
	Matrix *t;
   ~KalmanAngular();
   KalmanAngular( int numStates, int numMeasurements );
   void filter();
	void init();
private:
	float dt;
};
//---------------------------------------------------------------------------
#endif
