//---------------------------------------------------------------------------
#pragma hdrstop
#include "UnitMain.h"
#include "Kalman.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
Kalman::~Kalman()
{
   delete A;
   delete H;
   delete Q;
   delete R;
   delete x;
   delete xp;
   delete P;
   delete Pp;
   delete z;
   delete t;
   delete K;
}
//---------------------------------------------------------------------------
Kalman::Kalman( int numStates, int numMeasurements )
{
   A = new Matrix( numStates, numStates );
   H = new Matrix( numMeasurements, numStates );
   Q = new Matrix( numStates, numStates );
   R = new Matrix( numMeasurements, numMeasurements );
   x = new Matrix( numStates, 1 );
   xp = new Matrix( numStates, 1 );
   P = new Matrix( numStates, numStates );
   Pp = new Matrix( numStates, numStates );
   z = new Matrix( numMeasurements, 1 );
   t = new Matrix( numMeasurements, numMeasurements );
   K = new Matrix( numStates, numStates );
}
//---------------------------------------------------------------------------
void Kalman::filter()
{
   (*xp) = (*A) * (*x);
   (*Pp) = (*A) * (*P) * (~(*A)) + (*Q);
   (*t) = (*H) * (*Pp) * (~(*H)) + (*R);
   (*K) = (*Pp) * (~(*H)) * t->inv( (*t) );
   (*x) = (*xp) + (*K) * ( (*z) - (*H) * (*xp) );
   (*P) = (*Pp) - (*K) * (*H) * (*Pp);
}
//---------------------------------------------------------------------------
