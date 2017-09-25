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
	float pz, dz, t;
	Point3 hP;
	pz = ray.p.z;
	dz = ray.dir.z;

	if (dz == 0)
		return false;

	t = -(pz / dz);
	if (t < hInfo.z && t > BIAS) {
		hP = ray.p + t * ray.dir;
		if (fabs(hP.x) < 1 && fabs(hP.y) < 1)
		{
			hInfo.z = t;
			hInfo.N = Point3(0, 0, 1);
			hInfo.p = hP;
			return true;
		}
	}
	return false;
}

//assume box is axis aligned
//check if ray is aligned to any of the axis and then do ray plane interaction with the remaining faces.'
bool Box::IntersectRay(const Ray &r, float t_max) const {

	if (IsEmpty())
		return false;
	float ltx, lty, ltz, tmin = BIGFLOAT; // lower bound x y z
	float utx, uty, utz, tmax = 0; // upper bound x y z

	if (r.dir.x != 0)
	{
		if (r.dir.x > 0)
		{
			tmin = ltx = (pmin.x - r.p.x) / r.dir.x;
			tmax = utx = (pmax.x - r.p.x) / r.dir.x;
		}
		else
		{
			tmax = utx = (pmin.x - r.p.x) / r.dir.x;
			tmin = ltx = (pmax.x - r.p.x) / r.dir.x;
		}
	}
	if (r.dir.y != 0)
	{
		if (r.dir.y > 0)
		{
			lty = (pmin.y - r.p.y) / r.dir.y;
			uty = (pmax.y - r.p.y) / r.dir.y;
		}
		else
		{
			 uty = (pmin.y - r.p.y) / r.dir.y;
			 lty = (pmax.y - r.p.y) / r.dir.y;
		}

		if ((tmin > uty) || (lty > tmax))
			return false;
		
		if (tmin < lty)
			tmin = lty;
		
		if (tmax > uty)
			tmax = uty;
	}
	if (r.dir.z != 0)
	{
		if (r.dir.z > 0)
		{
			ltz = (pmin.z - r.p.z) / r.dir.z;
			utz = (pmax.z - r.p.z) / r.dir.z;
		}
		else
		{
			utz = (pmin.z - r.p.z) / r.dir.z;
			ltz = (pmax.z - r.p.z) / r.dir.z;
		}
		if ((tmin > utz) || (ltz > tmax))
			return false;

		if (tmin < ltz)
			tmin = ltz;
		
		if (tmax > utz)
			tmax = utz;
	}
	
	return true;
}

bool TriObj::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {
	int faces = NF();
	bool status = false;
	Box boundingBox = GetBoundBox();
	if (boundingBox.IntersectRay(ray, hInfo.z))
	{
		for (int i = 0; i < faces; i++)
		{
			float z = hInfo.z;
			if (IntersectTriangle(ray, hInfo, hitSide, i))
			{
				status = true;
			}
		}
	}
	return status;
}

bool TriObj::IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const
{
	float t = BIGFLOAT;
	Point3 vertexA, vertexB, vertexC, triNUnNorm, triNormal, hitPoint;
		
	TriFace face = F(faceID);
	vertexA = V(face.v[0]);
	vertexB = V(face.v[1]);
	vertexC = V(face.v[2]);

	triNUnNorm = (vertexB - vertexA).Cross(vertexC - vertexA);
	triNormal = triNUnNorm.GetNormalized();
		
	//t = (d - n.P)/n.D; d = n.x (x - any point on triangle eg. A => d = n.A); t = (n.A - n.P)/n.D;
	if (ray.dir.Dot(triNormal) != 0)
		t = (triNormal.Dot(vertexA) - (triNormal.Dot(ray.p))) / triNormal.Dot(ray.dir);

	if (t < hInfo.z && t > BIAS)
	{
		hitPoint = ray.p + t * ray.dir;
		//checking if it is inside the triangle
		float totalArea, APB, BPC, CPA, alpha, beta, gamma;
		totalArea = triNUnNorm.Dot(triNormal);
		APB = ((vertexB - vertexA).Cross(hitPoint - vertexA)).Dot(triNormal);
		BPC = ((vertexC - vertexB).Cross(hitPoint - vertexB)).Dot(triNormal);
		CPA = ((vertexA - vertexC).Cross(hitPoint - vertexC)).Dot(triNormal);
		alpha = BPC / totalArea;
		gamma = APB / totalArea;
		beta = CPA / totalArea;

		if (alpha >= 0 && beta >= 0 && gamma >= 0 )
		{
			//inside the circle
			hInfo.p = hitPoint;
			hInfo.z = t;
			Point3 intrapolatedNormal = GetNormal(faceID, Point3(alpha, beta, gamma));
			hInfo.N = intrapolatedNormal;
			return true;
		}
	}
	return false;
}