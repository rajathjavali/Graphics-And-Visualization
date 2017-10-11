/*
*  Sphere.cpp
*  Created on: Sept 21, 2017
*      Author: RajathJavali
*/

#include "objects.h"

#define BIAS 0.001

//#define min(x, y) ((x) < (y) ? (x) : (y))
//#define max(x, y) ((x) > (y) ? (x) : (y))

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
			if (ray.dir.Dot(Point3(0, 0, 1)) > 0)
				hInfo.N = Point3(0, 0, 1);
			else
				hInfo.N = Point3(0, 0, -1);

			hInfo.p = hP;
			return true;
		}
	}
	return false;
}

//assume box is axis aligned
//check if ray is aligned to any of the axis and then do ray plane interaction with the remaining faces.'
bool Box::IntersectRay(const Ray &r, float t_max) const {
	
	bool intersectStatus = false;
	if (IsEmpty())
		return false;
	float val1 = 0, val2 = 0;
	float x0, y0, z0, tmin = -INFINITY; // lower bound x y z
	float x1, y1, z1, tmax = t_max; // upper bound x y z

	float dirInv = 0;

	if (r.dir.x != 0)
	{
		dirInv = 1.0f / r.dir.x;
		
		val1 = (pmin.x - r.p.x) * dirInv;
		val2 = (pmax.x - r.p.x) * dirInv;

		x0 = min(val1, val2);
		x1 = max(val1, val2);
		
		tmin = max(tmin, min(x0, x1));
		tmax = min(tmax, max(x0, x1));

		intersectStatus = true;
	}
	if (r.dir.y != 0)
	{
		dirInv = 1.0f / r.dir.y;

		val1 = (pmin.y - r.p.y) * dirInv;
		val2 = (pmax.y - r.p.y) * dirInv;

		y0 = min(val1, val2);
		y1 = max(val1, val2);

		if ((tmin > y1) || (y0 > tmax))
			return false;
		
		tmin = max(tmin, y0);
		tmax = min(tmax, y1);

		intersectStatus = true;
	}
	if (r.dir.z != 0)
	{
		dirInv = 1.0f / r.dir.z;

		val1 = (pmin.z - r.p.z) * dirInv;
		val2 = (pmax.z - r.p.z) * dirInv;

		z0 = min(val1, val2);
		z1 = max(val1, val2);

		if ((tmin > z1) || (z0 > tmax))
			return false;

		tmin = max(tmin, z0);
		tmax = min(tmax, z1);

		intersectStatus = true;
	}
	
	if(tmin <= tmax)
		return intersectStatus;
	return false;

}

unsigned int pop(std::vector<unsigned int>& stack) {
	unsigned int i = 0;
	if (!stack.empty())
	{
		i = stack.back();
		stack.pop_back();
	}
	return i;
}

bool TriObj::IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide) const {

	bool status = false;
	Box boundingBox = GetBoundBox();
	unsigned int nodeID, child1 = 0, child2 = 0, rootID = bvh.GetRootNodeID();
	std::vector<unsigned int> stack;
	stack.push_back(rootID);

	while (!stack.empty())
	{
		if (!bvh.IsLeafNode(nodeID = pop(stack)))
		{
			boundingBox = bvh.GetNodeBounds(nodeID);
			if (boundingBox.IntersectRay(ray, hInfo.z))
			{
				bvh.GetChildNodes(nodeID, child1, child2);
				stack.push_back(child2), stack.push_back(child1);
			}
		}
		else {
			boundingBox = bvh.GetNodeBounds(nodeID);
			if (boundingBox.IntersectRay(ray, hInfo.z))
				if (TraceBVHNode(ray, hInfo, hitSide, nodeID))
					status = true;
		}
	}

	return status;
}

bool TriObj::TraceBVHNode(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID) const
{
	bool status = false;
	int count = bvh.GetNodeElementCount(nodeID);
	const unsigned int* list = bvh.GetNodeElements(nodeID);
	for (int i = 0; i < count; i++)
	{
		float z = hInfo.z;
		if (IntersectTriangle(ray, hInfo, hitSide, list[i]))
		{
			if (z > hInfo.z)
				status = true;
		}
	}
	return status;
}

bool TriObj::IntersectTriangle(const Ray &ray, HitInfo &hInfo, int hitSide, unsigned int faceID) const
{
	float t = BIGFLOAT;
	Point3 vertexA, vertexB, vertexC, triNormal, hitPoint;
		
	TriFace face = F(faceID);
	vertexA = V(face.v[0]);
	vertexB = V(face.v[1]);
	vertexC = V(face.v[2]);

	triNormal = (vertexB - vertexA).Cross(vertexC - vertexA);
	triNormal.Normalize();
	
	if (hitSide == HIT_BACK)
	{
		if (ray.dir.Dot(triNormal) < 0)
			return false;
	}
	if (hitSide == HIT_FRONT)
	{
		if (ray.dir.Dot(triNormal) > 0)
			return false;
	}

	//t = (d - n.P)/n.D; d = n.x (x - any point on triangle eg. A => d = n.A); t = (n.A - n.P)/n.D;
	if (ray.dir.Dot(triNormal) != 0)
		t = (triNormal.Dot(vertexA) - (triNormal.Dot(ray.p))) / triNormal.Dot(ray.dir);

	if (t < hInfo.z && t > BIAS)
	{
		hitPoint = ray.p + t * ray.dir;

		//projecting everything into 2d in order to optimize the calculation
		Point2 vertA2d, vertB2d, vertC2d, triNUnNorm2d, triNormal2d, hitPoint2d;
		float totalArea, APB, BPC, CPA, alpha, beta, gamma;
		if (triNormal.x > triNormal.y && triNormal.x > triNormal.z)
		{
			vertA2d = Point2(vertexA.y, vertexA.z);
			vertB2d = Point2(vertexB.y, vertexB.z);
			vertC2d = Point2(vertexC.y, vertexC.z);
			hitPoint2d = Point2(hitPoint.y, hitPoint.z);
		}
		else if (triNormal.y > triNormal.x && triNormal.y > triNormal.z)
		{
			vertA2d = Point2(vertexA.x, vertexA.z);
			vertB2d = Point2(vertexB.x, vertexB.z);
			vertC2d = Point2(vertexC.x, vertexC.z);
			hitPoint2d = Point2(hitPoint.x, hitPoint.z);
		}
		else
		{
			vertA2d = Point2(vertexA.x, vertexA.y);
			vertB2d = Point2(vertexB.x, vertexB.y);
			vertC2d = Point2(vertexC.x, vertexC.y);
			hitPoint2d = Point2(hitPoint.x, hitPoint.y);
		}
		totalArea = (vertB2d - vertA2d).Cross(vertC2d - vertA2d);
		APB = (vertB2d - vertA2d).Cross(hitPoint2d - vertA2d);
		BPC = (vertC2d - vertB2d).Cross(hitPoint2d - vertB2d);
		//CPA = (vertA2d - vertC2d).Cross(hitPoint2d - vertC2d);
		alpha = BPC / totalArea;
		gamma = APB / totalArea;
		beta = 1 - (alpha + gamma);
		//beta = CPA / totalArea;
		if (alpha >= 0 && beta >= 0 && gamma >= 0)
		{
			//inside the circle
			hInfo.p = hitPoint;
			hInfo.z = t;
			Point3 intrapolatedNormal = GetNormal(faceID, Point3(alpha, beta, gamma));
			intrapolatedNormal.Normalize();
			if (ray.dir.Dot(intrapolatedNormal) > 0)
				hInfo.N = intrapolatedNormal;
			else
				hInfo.N = -intrapolatedNormal;
			return true;
		}
	}
	return false;
}