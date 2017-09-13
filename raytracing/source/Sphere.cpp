/*
*  Sphere.cpp
*  Created on: Aug 25, 2017
*      Author: RajathJavali
*/

#include "objects.h"

#define BIAS 0.001

Sphere theSphere;

/* Equation for intersection: d*d x^2 + 2*(ray-centerOfObj)*d x + (ray-centerOfObj) * (ray-centerOfObj) - radius^2 = 0
*  d -> direction from ray to intersection point
*  x -> distance from ray to intersection point (the z buffer)
*  before computing x, delta must be 0 or +ve -> b^2 - 4ac (since the abv equ is quadratic in x) */
bool Sphere::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const
{
	Point3  rayToObj; // rayToObj = ray - centerOfObj
	float a, b, c, delta;
	bool status = false;
	//not required since we are tracing in model space where the obj is of unit size.
	// checking if all the magnitude across the diagonal is the same 
	/*if ((hInfo.node->GetTransform().GetRow(0).x == hInfo.node->GetTransform().GetRow(1).y) &&
		(hInfo.node->GetTransform().GetRow(1).y == hInfo.node->GetTransform().GetRow(2).z)) 
		radius = hInfo.node->GetTransform().GetRow(0).x;*/

	rayToObj = ray.p;
	a = ray.dir.Dot(ray.dir);
	b = 2 * rayToObj.Dot(ray.dir);
	c = rayToObj.Dot(rayToObj) - 1.0;
	
	delta = b * b - 4 * a * c;
	if (delta < 0)
		return false;

	float x1, x2;
	x1 = (sqrt(delta) - b) / (2 * a);
	x2 = (-1 * sqrt(delta) - b) / (2 * a);

	/*if (x1 < 0 && x2 < 0)
		return false;*/

	if (hitSide == HIT_FRONT) {
		if (hInfo.z > x1 && x1 > 0 && x1 > BIAS)
		{
			hInfo.z = x1;
			hInfo.p = ray.p + x1 * ray.dir;
			hInfo.N = -hInfo.p;
			status = true;
		}
		if (hInfo.z > x2 && x2 > 0 && x2 > BIAS)
		{
			hInfo.z = x2;
			hInfo.p = ray.p + x2 * ray.dir;
			hInfo.N = -hInfo.p;
			status = true;
		}
	}
	else if (hitSide == HIT_BACK) {
		if (hInfo.z > x1 && x1 < 0 && x1 < -BIAS)
		{
			hInfo.z = x1;
			hInfo.p = ray.p + x1 * ray.dir;
			status = true;
		}
		if (hInfo.z > x2 && x2 < -BIAS)
		{
			hInfo.z = x2;
			hInfo.p = ray.p + x2 * ray.dir;
			status = true;
		}
	}
	else if (hitSide == HIT_FRONT_AND_BACK) {
		if(((x1 < 0 && hInfo.z > -x1 && -x1 > BIAS) || (x1 > 0 && hInfo.z > x1 && x1 > BIAS)))
		{
			if (x1 < 0)
				x1 = -x1;
			hInfo.z = x1;
			hInfo.p = ray.p + x1 * ray.dir;
			status = true;
		}
		if (((x2 < 0 && hInfo.z > -x2 && -x2 > BIAS) || (x2 > 0 && hInfo.z > x2 && x2 > BIAS)))
		{
			if (x2 < 0)
				x2 = -x2;
			hInfo.z = x2;
			hInfo.p = ray.p + x2 * ray.dir;
			status = true;
		}
	}

	return status;

}