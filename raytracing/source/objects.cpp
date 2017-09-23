/*
*  Sphere.cpp
*  Created on: Sept 21, 2017
*      Author: RajathJavali
*/

#include "objects.h"

#define BIAS 0.001

Plane thePlane;
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

	if (x1 < 0 && x2 < 0)
		return false;

	// smallest val is always the front hit
	if (hitSide == HIT_FRONT) {
		if (x1 > BIAS && x1 < x2 && hInfo.z > x1)
		{
			hInfo.z = x1;
			hInfo.p = ray.p + x1 * ray.dir;
			hInfo.N = -hInfo.p;
			hInfo.front = true;
			status = true;
		}
		else if (x2 < x1 && hInfo.z > x2 && x2 > BIAS)
		{
			hInfo.z = x2;
			hInfo.p = ray.p + x2 * ray.dir;
			hInfo.N = -hInfo.p;
			hInfo.front = true;
			status = true;
		}
	}
	else if (hitSide == HIT_BACK) {
		if (x1 > x2 && hInfo.z > x1 && x1 > BIAS)
		{
			hInfo.z = x1;
			hInfo.p = ray.p + x1 * ray.dir;
			hInfo.front = false;
			status = true;
		}
		else if (x2 > x1 && hInfo.z > x2 && x2 > BIAS)
		{
			hInfo.z = x2;
			hInfo.p = ray.p + x2 * ray.dir;
			hInfo.front = false;
			status = true;
		}
	}
	else if (hitSide == HIT_FRONT_AND_BACK) {
		if (hInfo.z > x1 && x1 > BIAS)
		{
			hInfo.z = x1;
			hInfo.p = ray.p + x1 * ray.dir;
			hInfo.N = -hInfo.p;
			if (x1 < x2)
				hInfo.front = true;
			else
				hInfo.front = false;
			status = true;
		}
		if (hInfo.z > x2 && x2 > BIAS)
		{
			hInfo.z = x2;
			hInfo.p = ray.p + x2 * ray.dir;
			hInfo.N = -hInfo.p;
			if (x2 < x1)
				hInfo.front = true;
			else
				hInfo.front = false;
			status = true;
		}
	}

	return status;

}

//Assuming axis alligned to z. Hit point is givem by p+td = q; in z axis pz + t dz = 0;
bool Plane::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {
	Ray r(ray.p, ray.dir.GetNormalized());
	float pz, dz, t;
	Point2 rayPos(ray.p.x, ray.p.y), rayDir(ray.dir.x, ray.dir.y), hitPoint;
	Point3 hP;
	pz = ray.p.z;
	dz = ray.dir.z;

	if (dz == 0)
		return false;

	t = -(pz / dz);
	if (t < hInfo.z && t > BIAS) {
		hitPoint = rayPos + t * rayDir;
		hP = ray.p + t * ray.dir;
		if (fabs(hitPoint.x) < 1 && fabs(hitPoint.y) < 1)
		{
			hInfo.z = t;
			hInfo.N = Point3(0, 0, 1);
			//hInfo.p = Point3(hitPoint, 0);// ray.p.z);
			hInfo.p = hP;
			return true;
		}
	}
	return false;
}

bool Box::IntersectRay(const Ray &r, float t_max) const {
	
	Point3 A, B, C, D, E, F, G, H; // box faces: x0 = ACGE; x1 = BDFH; y0 = ABFE; y1 = CDHG; z0 = ABDC; z1 = EFHG;
	A = Corner(0);
	B = Corner(1);
	C = Corner(2);
	D = Corner(3);
	E = Corner(4);
	F = Corner(5);
	G = Corner(6);
	H = Corner(7);

	Point3 x0N, x1N, y0N, y1N, z0N, z1N; // normals to the faces: x = xN; y = yN; z = zN;
	x0N = ((E - A).Cross((C - A))).GetNormalized();
	x1N = ((D - B).Cross((F - B))).GetNormalized();
	y0N = ((E - A).Cross((B - A))).GetNormalized();
	y1N = ((G - C).Cross((D - C))).GetNormalized();
	z0N = ((C - A).Cross((B - A))).GetNormalized();
	z1N = ((F - E).Cross((G - E))).GetNormalized();

	float tx0 = 0, ty0 = 0, tz0 = 0, tx1 = BIGFLOAT, ty1 = BIGFLOAT, tz1 = BIGFLOAT; // hit distances with all the faces
	// t = -(S - P).N / d.N; N - normal, S - point on the face; d - direction of ray; P - position of ray

	if (r.dir.Dot(x0N) != 0)
		tx0 = -(A - r.p).Dot(x0N) / r.dir.Dot(x0N);
	if (r.dir.Dot(x1N) != 0)
		tx1 = -(B - r.p).Dot(x1N) / r.dir.Dot(x1N);
	if (r.dir.Dot(y0N) != 0)
		ty0 = -(A - r.p).Dot(y0N) / r.dir.Dot(y0N);
	if (r.dir.Dot(y1N) != 0)
		ty1 = -(C - r.p).Dot(y1N) / r.dir.Dot(y1N);
	if (r.dir.Dot(z0N) != 0)
		tz0 = -(A - r.p).Dot(z0N) / r.dir.Dot(z0N);
	if (r.dir.Dot(z1N) != 0)
		tz1 = -(E - r.p).Dot(z1N) / r.dir.Dot(z1N);

	float tentry, texit; // hit distances on box on entry and exit; if ( entry < exit ) ray intersects the box;

	tentry = max(max(tx0, ty0), tz0);
	texit = min(min(tx1, ty1), tz1);
	if (tentry < texit && tentry < t_max)
		return true;

	return false;
}

bool TriObj::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {
	int vertices = NV();
	int faces = NF();
	return false;
}

bool TriObj::IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const {
	return false;
}