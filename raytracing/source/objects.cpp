/*
*  Sphere.cpp
*  Created on: Sept 21, 2017
*      Author: RajathJavali
*/

#include "objects.h"

#define BIAS 0.001
#define pi 3.1415

//#define min(x, y) ((x) < (y) ? (x) : (y))
//#define max(x, y) ((x) > (y) ? (x) : (y))

Plane thePlane;
Sphere theSphere;

/* Equation for intersection: d*d x^2 + 2*(ray-centerOfObj)*d x + (ray-centerOfObj) * (ray-centerOfObj) - radius^2 = 0
*  d -> direction from ray to intersection point
*  x -> distance from ray to intersection point (the z buffer)
*  before computing x, delta must be 0 or +ve -> b^2 - 4ac (since the abv equ is quadratic in x) */
bool Sphere::IntersectRay(const DifRays &rays, HitInfo &hInfo, int hitSide) const
{
	Point3  rayToObj; // rayToObj = ray - centerOfObj
	float a, b, c, delta;
	bool status = false;
	//not required since we are tracing in model space where the obj is of unit size.
	// checking if all the magnitude across the diagonal is the same 
	/*if ((hInfo.node->GetTransform().GetRow(0).x == hInfo.node->GetTransform().GetRow(1).y) &&
	(hInfo.node->GetTransform().GetRow(1).y == hInfo.node->GetTransform().GetRow(2).z))
	radius = hInfo.node->GetTransform().GetRow(0).x;*/

	rayToObj = rays.ray.p;
	a = rays.ray.dir.Dot(rays.ray.dir);
	b = 2 * rayToObj.Dot(rays.ray.dir);
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
			hInfo.p = rays.ray.p + x1 * rays.ray.dir;
			hInfo.N = -hInfo.p;
			hInfo.front = true;
			status = true;

		}
		else if (x2 < x1 && hInfo.z > x2 && x2 > BIAS)
		{
			hInfo.z = x2;
			hInfo.p = rays.ray.p + x2 * rays.ray.dir;
			hInfo.N = -hInfo.p;
			hInfo.front = true;
			status = true;
		}
	}
	else if (hitSide == HIT_BACK) {
		if (x1 > x2 && hInfo.z > x1 && x1 > BIAS)
		{
			hInfo.z = x1;
			hInfo.p = rays.ray.p + x1 * rays.ray.dir;
			hInfo.front = false;
			status = true;
		}
		else if (x2 > x1 && hInfo.z > x2 && x2 > BIAS)
		{
			hInfo.z = x2;
			hInfo.p = rays.ray.p + x2 * rays.ray.dir;
			hInfo.front = false;
			status = true;
		}
	}
	else if (hitSide == HIT_FRONT_AND_BACK) {
		if (hInfo.z > x1 && x1 > BIAS)
		{
			hInfo.z = x1;
			hInfo.p = rays.ray.p + x1 * rays.ray.dir;
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
			hInfo.p = rays.ray.p + x2 * rays.ray.dir;
			hInfo.N = -hInfo.p;
			if (x2 < x1)
				hInfo.front = true;
			else
				hInfo.front = false;
			status = true;
		}
	}

	if (status)
	{
		float u = 0.5 - atan2(hInfo.p.x, hInfo.p.y) / (2 * pi);
		float v = 0.5 + asin(hInfo.p.z) / pi;
		hInfo.uvw = Point3(u, v, 0);

		if (rays.status) {
			Point3 rayToObjx, rayToObjy, rayToObjCent;
			Point3 hitx, hity, hitCent;

			rayToObjCent = rays.difcenter.p;
			a = rays.difcenter.dir.Dot(rays.difcenter.dir);
			b = 2 * rayToObjCent.Dot(rays.difcenter.dir);
			c = rayToObjCent.Dot(rayToObjCent) - 1.0;

			delta = b * b - 4 * a * c;
			if (delta > 0)
			{
				x1 = (sqrt(delta) - b) / (2 * a);
				x2 = (-1 * sqrt(delta) - b) / (2 * a);
				if (x1 > BIAS && x1 < x2)
					hitCent = rays.difcenter.p + x1 * rays.difcenter.dir;
				else
					hitCent = rays.difcenter.p + x2 * rays.difcenter.dir;
			}

			rayToObjx = rays.difx.p;
			a = rays.difx.dir.Dot(rays.difx.dir);
			b = 2 * rayToObjx.Dot(rays.difx.dir);
			c = rayToObjx.Dot(rayToObjx) - 1.0;

			delta = b * b - 4 * a * c;
			if (delta > 0)
			{
				x1 = (sqrt(delta) - b) / (2 * a);
				x2 = (-1 * sqrt(delta) - b) / (2 * a);
				if (x1 > BIAS && x1 < x2)
					hitx = rays.difx.p + x1 * rays.difx.dir;
				else
					hitx = rays.difx.p + x2 * rays.difx.dir;
			}

			rayToObjy = rays.dify.p;
			a = rays.dify.dir.Dot(rays.dify.dir);
			b = 2 * rayToObjy.Dot(rays.dify.dir);
			c = rayToObjy.Dot(rayToObjy) - 1.0;

			delta = b * b - 4 * a * c;
			if (delta > 0)
			{
				x1 = (sqrt(delta) - b) / (2 * a);
				x2 = (-1 * sqrt(delta) - b) / (2 * a);
				if (x1 > BIAS && x1 < x2)
					hity = rays.dify.p + x1 * rays.dify.dir;
				else
					hity = rays.dify.p + x2 * rays.dify.dir;
			}

			hInfo.duvw[0] = hitx - hitCent;
			hInfo.duvw[1] = hity - hitCent;
		}
	}
	return status;

}

//Assuming axis alligned to z. Hit point is givem by p+td = q; in z axis pz + t dz = 0;
bool Plane::IntersectRay(const DifRays &rays, HitInfo &hInfo, int hitSide) const {
	float pz, dz, t;
	Point3 hP;
	pz = rays.ray.p.z;
	dz = rays.ray.dir.z;

	if (dz == 0)
		return false;

	t = -(pz / dz);
	if (t < hInfo.z && t > BIAS) {
		hP = rays.ray.p + t * rays.ray.dir;
		if (fabs(hP.x) < 1 && fabs(hP.y) < 1)
		{
			hInfo.z = t;
			if (rays.ray.dir.Dot(Point3(0, 0, 1)) > 0)
				hInfo.N = Point3(0, 0, 1);
			else
				hInfo.N = Point3(0, 0, -1);

			hInfo.p = hP;
			hInfo.uvw = Point3((hInfo.p.x + 1) / 2, (hInfo.p.y + 1) / 2, 0);
			// ray differential sampling
			if(rays.status)
			{
				float pxz, dxz, tx = BIGFLOAT, pyz, dyz, ty = BIGFLOAT;
				Point3 hPx, hPy;
				pxz = rays.difx.p.z;
				dxz = rays.dify.dir.z;
				
				if (dxz != 0) {
					tx = -(pxz / dxz);
					hPx = rays.difx.p + tx * rays.difx.dir;
				}

				pyz = rays.dify.p.z;
				dyz = rays.dify.dir.z;
				
				if (dyz != 0) {
					ty = -(pyz / dyz);
					hPy = rays.dify.p + ty * rays.dify.dir;
				}
				hPx.x = fmin(1, hPx.x), hPx.y = fmin(1, hPx.y);
				hPy.x = fmin(1, hPy.x), hPy.y = fmin(1, hPy.y);

				hInfo.duvw[0] = (hPx - hP) * 1.5;
				hInfo.duvw[1] = (hPy - hP) * 1.5;
			}

			return true;
		}
	}
	return false;
}

//assume box is axis aligned
//check if ray is aligned to any of the axis and then do ray plane interaction with the remaining faces.'
bool Box::IntersectRay(const DifRays &r, float t_max) const {
	
	bool intersectStatus = false;
	if (IsEmpty())
		return false;
	float val1 = 0, val2 = 0;
	float x0, y0, z0, tmin = -INFINITY; // lower bound x y z
	float x1, y1, z1, tmax = t_max; // upper bound x y z

	float dirInv = 0;

	if (r.ray.dir.x != 0)
	{
		dirInv = 1.0f / r.ray.dir.x;
		
		val1 = (pmin.x - r.ray.p.x) * dirInv;
		val2 = (pmax.x - r.ray.p.x) * dirInv;

		x0 = min(val1, val2);
		x1 = max(val1, val2);
		
		tmin = max(tmin, min(x0, x1));
		tmax = min(tmax, max(x0, x1));

		intersectStatus = true;
	}
	if (r.ray.dir.y != 0)
	{
		dirInv = 1.0f / r.ray.dir.y;

		val1 = (pmin.y - r.ray.p.y) * dirInv;
		val2 = (pmax.y - r.ray.p.y) * dirInv;

		y0 = min(val1, val2);
		y1 = max(val1, val2);

		if ((tmin > y1) || (y0 > tmax))
			return false;
		
		tmin = max(tmin, y0);
		tmax = min(tmax, y1);

		intersectStatus = true;
	}
	if (r.ray.dir.z != 0)
	{
		dirInv = 1.0f / r.ray.dir.z;

		val1 = (pmin.z - r.ray.p.z) * dirInv;
		val2 = (pmax.z - r.ray.p.z) * dirInv;

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

bool TriObj::IntersectRay(const DifRays &rays, HitInfo &hInfo, int hitSide) const {

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
			if (boundingBox.IntersectRay(rays, hInfo.z))
			{
				bvh.GetChildNodes(nodeID, child1, child2);
				stack.push_back(child2), stack.push_back(child1);
			}
		}
		else {
			boundingBox = bvh.GetNodeBounds(nodeID);
			if (boundingBox.IntersectRay(rays, hInfo.z))
				if (TraceBVHNode(rays, hInfo, hitSide, nodeID))
					status = true;
		}
	}

	return status;
}

bool TriObj::TraceBVHNode(const DifRays &rays, HitInfo &hInfo, int hitSide, unsigned int nodeID) const
{
	bool status = false;
	int count = bvh.GetNodeElementCount(nodeID);
	const unsigned int* list = bvh.GetNodeElements(nodeID);
	for (int i = 0; i < count; i++)
	{
		float z = hInfo.z;
		if (IntersectTriangle(rays, hInfo, hitSide, list[i]))
		{
			if (z > hInfo.z)
				status = true;
		}
	}
	return status;
}

bool TriObj::IntersectTriangle(const DifRays &rays, HitInfo &hInfo, int hitSide, unsigned int faceID) const
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
		if (rays.ray.dir.Dot(triNormal) < 0)
			return false;
	}
	if (hitSide == HIT_FRONT)
	{
		if (rays.ray.dir.Dot(triNormal) > 0)
			return false;
	}

	//t = (d - n.P)/n.D; d = n.x (x - any point on triangle eg. A => d = n.A); t = (n.A - n.P)/n.D;
	if (rays.ray.dir.Dot(triNormal) != 0)
		t = (triNormal.Dot(vertexA) - (triNormal.Dot(rays.ray.p))) / triNormal.Dot(rays.ray.dir);

	if (t < hInfo.z && t > BIAS)
	{
		hitPoint = rays.ray.p + t * rays.ray.dir;

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
			if (rays.ray.dir.Dot(intrapolatedNormal) > 0)
				hInfo.N = intrapolatedNormal;
			else
				hInfo.N = -intrapolatedNormal;

			hInfo.uvw = GetTexCoord(faceID, Point3(alpha, beta, gamma));
			// ray differential sampling
			if (rays.status)
			{
				float tx = BIGFLOAT, ty = BIGFLOAT;
				Point3 hitPointx, hitPointy, dx, dy;

				if (rays.difx.dir.Dot(triNormal) != 0)
					tx = (triNormal.Dot(vertexA) - (triNormal.Dot(rays.difx.p))) / triNormal.Dot(rays.difx.dir);

				if (rays.dify.dir.Dot(triNormal) != 0)
					ty = (triNormal.Dot(vertexA) - (triNormal.Dot(rays.dify.p))) / triNormal.Dot(rays.dify.dir);

				hitPointx = rays.difx.p + tx * rays.difx.dir;
				hitPointy = rays.dify.p + ty * rays.dify.dir;
				dx = hitPointx - hitPoint;
				dy = hitPointy - hitPoint;

				hInfo.duvw[0] = dx / 2;
				hInfo.duvw[1] = dy / 2;
			}
			hInfo.mtlID = GetMaterialIndex(faceID);
			return true;
		}
	}
	return false;
}