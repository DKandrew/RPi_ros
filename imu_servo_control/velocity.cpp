#include "velocity.h"

/*////////////////////////////////////////////////////////////////////
 * This .cpp file contains functions required for velocity Calculation
/*////////////////////////////////////////////////////////////////////

Velocity::Velocity(float interval, float x, float y, float z)
{
	dt = interval;
	offX = x;
	offY = y;
	offZ = z;
	
	for(int i = 0; i < 3; ++i)
	{
		prevA[i] = 0;
		prevV[i] = 0;
		currV[i] = 0;
	}
}

void Velocity::updateVel(float currAx, float currAy, float currAz)
{
	currV[0] = prevV[0] + dt*(prevA[0] + currAx)/2;
	currV[1] = prevV[1] + dt*(prevA[1] + currAy)/2;
	currV[2] = prevV[2] + dt*(prevA[2] + currAz)/2;
	

	prevV[0] = currV[0];
	prevV[1] = currV[1];
	prevV[2] = currV[2];
	
	prevA[0] = currAx;
	prevA[1] = currAy;
	prevA[2] = currAz;
}

void Velocity::transform(float wx, float wy, float wz)
{
	float X[3] = {1,0,0};
	float Y[3] = {0,1,0};
	float Z[3] = {0,0,1};
	float O[3] = {offX, offY, offZ};
	
	//float crossX[3] ={X[1]*O[2] - X[2]*O[1], X[2]*O[0] - X[0]*O[2], X[0]*O[1] - X[1]*O[0]};
	//float crossY[3] ={Y[1]*O[2] - Y[2]*O[1], Y[2]*O[0] - Y[0]*O[2], Y[0]*O[1] - Y[1]*O[0]};
	//float crossZ[3] ={Z[1]*O[2] - Z[2]*O[1], Z[2]*O[0] - Z[0]*O[2], Z[0]*O[1] - Z[1]*O[0]};
	
	//currV[0] -= crossX[0]*wx + crossY[0]*wy + crossZ[0]*wz;
	//currV[1] -= crossX[1]*wx + crossY[1]*wy + crossZ[1]*wz;
	//currV[2] -= crossX[2]*wx + crossY[2]*wy + crossZ[2]*wz;
	
	float rotX = offX*wy; 
	float rotY = offY*wz;
	float rotZ = offZ*wx;
	
	currV[0] -= rotX;
	currV[1] -= rotY;
	currV[2] -= rotZ;
	
	
}

float* Velocity::getV()
{
	return currV;
}
