//---------------------------------------------------------------------------
#pragma hdrstop
#include "KalmanAngular.h"
//---------------------------------------------------------------------------
#pragma package(smart_init)
//---------------------------------------------------------------------------
KalmanAngular::~KalmanAngular()
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
KalmanAngular::KalmanAngular( int numStates, int numMeasurements )
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
	dt = Globals::T;
}
//---------------------------------------------------------------------------
void KalmanAngular::filter()
{
	(*xp) = (*A) * (*x);
	(*Pp) = (*A) * (*P) * (~(*A)) + (*Q);
	(*t) = (*H) * (*Pp) * (~(*H)) + (*R);
	(*K) = (*Pp) * (~(*H)) * t->inv( (*t) );
	(*x) = (*xp) + (*K) * ( (*z) - (*H) * (*xp) );
	(*P) = (*Pp) - (*K) * (*H) * (*Pp);
}
//---------------------------------------------------------------------------
void KalmanAngular::init()
{
	A->val[ 0 ][ 0 ] = 1.0;
	A->val[ 0 ][ 1 ] = dt;
	A->val[ 0 ][ 2 ] = 0.5 * dt * dt;
	A->val[ 1 ][ 0 ] = 0.0;
	A->val[ 1 ][ 1 ] = 1.0;
	A->val[ 1 ][ 2 ] = dt;
	A->val[ 2 ][ 0 ] = 0.0;
	A->val[ 2 ][ 1 ] = 0.0;
	A->val[ 2 ][ 2 ] = 1.0;

	H->val[ 0 ][ 0 ] = 1.0;
	H->val[ 0 ][ 1 ] = 0.0;
	H->val[ 0 ][ 2 ] = 0.0;
	H->val[ 1 ][ 0 ] = 0.0;
	H->val[ 1 ][ 1 ] = 1.0;
	H->val[ 1 ][ 2 ] = 0.0;

	Q->val[ 0 ][ 0 ] = 1.0;
	Q->val[ 0 ][ 1 ] = 0.0;
	Q->val[ 0 ][ 2 ] = 0.0;
	Q->val[ 1 ][ 0 ] = 0.0;
	Q->val[ 1 ][ 1 ] = 3.0;
	Q->val[ 1 ][ 2 ] = 0.0;
	Q->val[ 2 ][ 0 ] = 0.0;
	Q->val[ 2 ][ 1 ] = 0.0;
	Q->val[ 2 ][ 2 ] = 1.0;

	R->val[ 0 ][ 0 ] = 0.1;
	R->val[ 0 ][ 1 ] = 0.0;
	R->val[ 1 ][ 0 ] = 0.0;
	R->val[ 1 ][ 1 ] = 0.1;

	x->val[ 0 ][ 0 ] = 0.0;
	x->val[ 1 ][ 0 ] = 0.0;
	x->val[ 2 ][ 0 ] = 0.0;

	P->val[ 0 ][ 0 ] = 5.0;
	P->val[ 0 ][ 1 ] = 0.0;
	P->val[ 0 ][ 2 ] = 0.0;
	P->val[ 1 ][ 0 ] = 0.0;
	P->val[ 1 ][ 1 ] = 5.0;
	P->val[ 1 ][ 2 ] = 0.0;
	P->val[ 2 ][ 0 ] = 0.0;
	P->val[ 2 ][ 1 ] = 0.0;
	P->val[ 2 ][ 2 ] = 5.0;
}
//---------------------------------------------------------------------------
