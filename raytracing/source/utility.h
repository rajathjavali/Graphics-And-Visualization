#pragma once

#define PI 3.141592653589793
#define MIN_SAMPLES 8
#define MAX_SAMPLES 64
#define RAY_DIFFERENTIAL false
#define VARIANCE 0.001
#define ADAPTIVE true
#define Halton_Random_Sampling true //false uses random, true uses halton for camera ray position
#define SHADOW_SAMPLING true
#define MAX_RAY_BOUNCE_COUNT 16
#define MAX_GI_BOUNCE_COUNT 1
#define GI_MAX_RAYS 10
#define GI_MIN_RAYS 1

void Init();
void Trace(int i, int j);

Point3 getHaltonNormal(float radius, HitInfo hInfo, int raySampleCount)
{
	Point3 newNormal;
	float x, y, z, xdist, ydist, zdist, dist;
	Point3 xAxis = Point3(1, 0, 0), yAxis = Point3(0, 1, 0), zAxis(0, 0, 1);
	Point3 sphPoint, sphCent = hInfo.p + hInfo.N.GetNormalized(),
		sphBound = sphCent - radius * xAxis - radius * yAxis + radius * zAxis;

	bool status = false;
	float sqRadius = radius * radius;
	do {
		status = false;
		x = Halton(raySampleCount, 2); // Halton sequence base 2
		y = Halton(raySampleCount, 3); // Halton sequence base 3
		z = Halton(raySampleCount, 5); // Halton sequence base 5
		xdist = x * radius * 2;
		ydist = y * radius * 2;
		zdist = z * radius * 2;
		Point3 newPoint = sphBound + xdist * xAxis + ydist * yAxis - zdist * zAxis;

		dist = pow(sphCent.x - newPoint.x, 2) + pow(sphCent.y - newPoint.y, 2) + pow(sphCent.z - newPoint.z, 2);
		if (dist <= sqRadius)
			status = true, newNormal = (newPoint - hInfo.p).GetNormalized();
		else
			raySampleCount++;
	} while (!status && raySampleCount < 1000);

	return newNormal;
}