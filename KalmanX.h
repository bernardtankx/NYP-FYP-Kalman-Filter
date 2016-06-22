//---------------------------------------------------------------------------
#ifndef KalmanXH
#define KalmanXH
#include "Globals.h"
#include "matrix.h"

//---------------------------------------------------------------------------
class KalmanX {
public:
   Matrix *A, *H, *Q, *R;
	Matrix *x, *P, *xp, *Pp, *K, *z;
	Matrix *t;
	~KalmanX();
	KalmanX( int numStates, int numMeasurements );
   void filter();
	void init();
private:
	float dt;
};
//---------------------------------------------------------------------------
#endif
