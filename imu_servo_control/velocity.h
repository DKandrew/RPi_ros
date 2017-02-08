#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace std;

class Velocity
{
	private:
		float prevA[3];
		float prevV[3];
		float currV[3];
		float dt;
		float offX;
		float offY;
		float offZ;
		
	public:
		// consturctor
		Velocity(float interval, float x, float y, float z);
		// func
		void updateVel(float currAx, float currAy, float currAz);
		void transform(float wx, float wy, float wz);
		float* getV();
		
};
