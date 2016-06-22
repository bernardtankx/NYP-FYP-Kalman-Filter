//---------------------------------------------------------------------------
#ifndef KalmanH
#define KalmanH
#include "matrix.h"
//---------------------------------------------------------------------------
class Kalman {
public:
   Matrix *A, *H, *Q, *R;
   Matrix *x, *P, *xp, *Pp, *K, *z;
   Matrix *t;
   ~Kalman();
   Kalman( int numStates, int numMeasurements );
   void filter();
};
//---------------------------------------------------------------------------
#endif
