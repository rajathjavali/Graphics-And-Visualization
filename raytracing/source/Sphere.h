/*
*  Sphere.h
*  Created on: Aug 25, 2017
*      Author: RajathJavali
*/

#pragma once
#include "scene.h"
#include <math.h>

class Sphere : public Object
{
	Sphere() {};

	bool IntersectRay(const Ray &ray, HitInfo &hInfo, int hitSide = HIT_FRONT) const;
	void ViewportDisplay() const {} // used for OpenGL display
};