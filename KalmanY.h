//---------------------------------------------------------------------------
#ifndef KalmanYH
#define KalmanYH
#include "Globals.h"
#include "matrix.h"

//---------------------------------------------------------------------------
class KalmanY {
public:
	Matrix *A, *H, *Q, *R;
	Matrix *x, *P, *xp, *Pp, *K, *z;
	Matrix *t;
	~KalmanY();
   KalmanY( int numStates, int numMeasurements );
   void filter();
	void init();
private:
	float dt;
};
//---------------------------------------------------------------------------
#endif
