/*
*  raytracing.cpp
*  Created on: Aug 25, 2017
*      Author: RajathJavali
*/
// raytracing.cpp : Defines the entry point for the console application.
#include "scene.h"
#include "Sphere.h"
#include "tbb/tbb.h"
#include "tbb/task_scheduler_init.h"
#include "materials.h"
#include "lights.h"

#include <thread>
#include <atomic>
#include <time.h>
#include <iostream>
#include <math.h>

using namespace tbb;

#define PI 3.1415
#define CAM_TO_IMG_DIST 1 // later gets changed, distance of img plane from the camera
#define IMG_NAME "prj5.jpg"
#define Z_IMG_NAME "prj5_z.jpg"
#define RESOURCE_NAME "resource\\Cornell_Box_Scene.xml" //scene_prj2.xml, simple_box_scene.xml, scene_prj3.xml, scene_prj4.xml, Cornell_Box_Scene.xml, example_box_tri_obj.xml

Node rootNode;
Camera camera;
RenderImage renderImage;
MaterialList materials;
LightList lights;
ObjFileList objList;

Point3 u, v, startingPoint;
float *distanceBuffer;
Color24 *pixelArray;

extern void ShowViewport();
extern int LoadScene(const char *filename);
void BeginRender();
void Trace(int i, int j);
bool traceNode(Node *node, Ray &ray, HitInfo &hitInfo, int hitSide);
void StopRender();

void Init()
{
	u = camera.dir ^ camera.up; // using the right hand thumb rule and cross product of unit vectors to get the direction in +x direction
	v = camera.dir ^ u; // v vector in -y direction
	int imgSize = renderImage.GetWidth() * renderImage.GetHeight();
	pixelArray = renderImage.GetPixels();

	float heightOfImgPlane = 2 * tan((camera.fov / 2) * PI / 180.0f) * CAM_TO_IMG_DIST;
	float pixelSize = heightOfImgPlane / camera.imgHeight;
	distanceBuffer = renderImage.GetZBuffer();

	// setting the magnitude of the vectors equivalent to pixel size	
	u = u * pixelSize;
	v = v * pixelSize;

	startingPoint = camera.pos + camera.dir * CAM_TO_IMG_DIST
		- v * (camera.imgHeight / 2) // since vector v is in -y direction and we need to go in +y direction to get to the top corner
		- u * (camera.imgWidth / 2); // since vector u is in +x direction and we need to go in -x direction to get to the top corner
}

// Blinn shading
// H = L+V/|L+V|
// I0(V) = Summation of all lights { Ii*(N*Li)(kd + ks (N*Hi)^a)} + Iamb * kd;  
// where I - intensity, N- normal to the material = ray.dir(if sphere), kd -diffusal const, ks - specular const, a - glossinesss
Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
	Color black = Color(0, 0, 0);
	if (bounceCount == 0)
		return black;

	if (!hInfo.front && refraction == Color(0, 0, 0))
		return black;

	Color color(0, 0, 0),
		absorptionCoeff(1, 1, 1),
		reflectedColor(0, 0, 0),
		refractedColor(0, 0, 0),
		totalReflection = reflection;

	Point3 Hi, surfaceNormal = hInfo.N.GetNormalized();
	Point3 viewDir = ray.dir.GetNormalized();

	float cosi, sini, sint, cost;

	for (int i = 0; i < lights.size(); i++)
	{
		Light *li = lights.at(i);
		Color liIntensity = li->Illuminate(hInfo.p, surfaceNormal);
		if (li->IsAmbient())
		{
			color += diffuse * liIntensity;
			continue;
		}

		Point3 lDir = li->Direction(hInfo.p).GetNormalized();


		Hi = (lDir + viewDir).GetNormalized(); // half vector		
		color += liIntensity * fabs(surfaceNormal.Dot(lDir)) * (diffuse + specular *  pow(fabs(surfaceNormal.Dot(Hi)), glossiness));
	}
	if (refraction != Color(0, 0, 0))
	{
		cosi = surfaceNormal.Dot(viewDir);
		sini = sqrt(1 - fmin(pow(cosi, 2), 1));

		HitInfo temp;
		temp.z = BIGFLOAT;
		//ray inside the object and getting a total internal reflection
		if (!hInfo.front) {
			sint = (float)sini * ior;
			if (sint > 1)
			{
				Point3 rayDir = (-2 * surfaceNormal * (surfaceNormal.Dot(viewDir)) + viewDir).GetNormalized();
				Ray totalReflectedRay(hInfo.p, rayDir);
				if (traceNode(&rootNode, totalReflectedRay, temp, HIT_FRONT_AND_BACK))
				{
					absorptionCoeff.r = exp(-hInfo.z * absorption.r);
					absorptionCoeff.g = exp(-hInfo.z * absorption.g);
					absorptionCoeff.b = exp(-hInfo.z * absorption.b);
					refractedColor += absorptionCoeff * temp.node->GetMaterial()->Shade(totalReflectedRay, temp, lights, (bounceCount - 1));
				}
			}
			else
			{
				cost = sqrt(1 - fmin(pow(sint, 2), 1));
				Point3 S = (surfaceNormal.Cross(surfaceNormal.Cross(-viewDir))).GetNormalized();
				Point3 refractedDir = (-surfaceNormal * cost + S * sint).GetNormalized();

				Ray refractedRay(hInfo.p, refractedDir);

				if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
				{
					if (temp.front)
						refractedColor += temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount -1));
				}
			}
		}
		else
		{
			sint = (float)sini / ior;

			float R0 = (float)(pow((1 - ior), 2)) / pow((1 + ior), 2);
			float reflectedPortion = R0 + (1 - R0) * pow((1 - cosi), 5);
			float refractedPortion = 1 - reflectedPortion;
			totalReflection += reflectedPortion * refraction;

			cost = sqrt(1 - fmin(pow(sint, 2), 1));
			Point3 S = (surfaceNormal.Cross(surfaceNormal.Cross(viewDir))).GetNormalized();
			Point3 refractedDir = (-1 * surfaceNormal * cost + S * sint).GetNormalized();
			Ray refractedRay(hInfo.p, -refractedDir);

			if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
			{
				if(!temp.front)
				{
					absorptionCoeff.r = exp(-temp.z * absorption.r);
					absorptionCoeff.g = exp(-temp.z * absorption.g);
					absorptionCoeff.b = exp(-temp.z * absorption.b);
					refractedColor += absorptionCoeff * refractedPortion * temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount-1));
				}
			}

		}
		color += refraction * refractedColor;
	}

	if (totalReflection != Color(0, 0, 0))
	{
		Point3 rayDir = -2 * surfaceNormal * (surfaceNormal.Dot(viewDir)) + viewDir;
		rayDir.Normalize();
		Ray reflectedRay(hInfo.p, rayDir);
		HitInfo temp;
		temp.z = BIGFLOAT;

		if (traceNode(&rootNode, reflectedRay, temp, HIT_FRONT))
			if (hInfo.front)
				color += totalReflection * temp.node->GetMaterial()->Shade(reflectedRay, temp, lights, (bounceCount - 1));
	}
	color.ClampMinMax();
	return color;

}

float GenLight::Shadow(Ray ray, float t_max)
{
	HitInfo hitInfo;
	hitInfo.z = t_max;
	if (t_max == 1) {
		float dist = sqrt(ray.dir.Dot(ray.dir));
		hitInfo.z = dist;
	}
	ray.Normalize();
	if (traceNode(&rootNode, ray, hitInfo, HIT_FRONT_AND_BACK))
		return 0;
	return 1;
}

int main()
{
	LoadScene(RESOURCE_NAME);

	std::thread *th = new std::thread(ShowViewport);
	/*
	u = camera.dir ^ camera.up; // using the right hand thumb rule and cross product of unit vectors to get the direction in +x direction
	v = camera.dir ^ u; // v vector in -y direction
	int imgSize = renderImage.GetWidth() * renderImage.GetHeight();
	pixelArray = renderImage.GetPixels();

	float heightOfImgPlane = 2 * tan((camera.fov / 2) * PI / 180.0f) * CAM_TO_IMG_DIST;
	float pixelSize = heightOfImgPlane / camera.imgHeight;
	distanceBuffer = renderImage.GetZBuffer();

	// setting the magnitude of the vectors equivalent to pixel size	
	u = u * pixelSize;
	v = v * pixelSize;

	startingPoint = camera.pos + camera.dir * CAM_TO_IMG_DIST
		- v * (camera.imgHeight / 2) // since vector v is in -y direction and we need to go in +y direction to get to the top corner
		- u * (camera.imgWidth / 2); // since vector u is in +x direction and we need to go in -x direction to get to the top corner

	clock_t startTime = clock();
	
	for (int j = 0; j < camera.imgHeight; j++)
		for (int i = 0; i < camera.imgWidth; i++)
			Trace(i, j);

	clock_t endTime = clock();

	double timeElapsed = static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
	printf("\nRender time is %f seconds\n", timeElapsed);

	
	renderImage.SaveImage(IMG_NAME);
	renderImage.ComputeZBufferImage();
	renderImage.SaveZImage(Z_IMG_NAME);
	*/
	printf("Enter a key to exit\n");
	while (getchar())
	{
		break;
	}

	return 1;
}

void BeginRender()
{
	Init();
	clock_t startTime = clock();
	tbb::parallel_for(0, camera.imgHeight, [&](int j) {
		for (int i = 0; i < camera.imgWidth; i++) {
			Trace(i, j);
		}
	});
	clock_t endTime = clock();

	double timeElapsed = static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
	printf("\nRender time is %f seconds\n", timeElapsed);

	renderImage.SaveImage(IMG_NAME);
	renderImage.ComputeZBufferImage();
	renderImage.SaveZImage(Z_IMG_NAME);
}

void Trace(int i, int j) {
	Point3 pixel_center_ij = startingPoint + (i + 0.5) * u + (j + 0.5) * v;
	Point3 dir = pixel_center_ij - camera.pos;
	Color pixelColour;
	pixelColour.SetBlack();

	Ray ray(camera.pos, dir);

	int pixelIndexInImg = j * camera.imgWidth + i;
	distanceBuffer[pixelIndexInImg] = BIGFLOAT;
	HitInfo hit_info;

	if (traceNode(&rootNode, ray, hit_info, HIT_FRONT))
	{
		distanceBuffer[pixelIndexInImg] = hit_info.z;
		pixelColour = hit_info.node->GetMaterial()->Shade(ray, hit_info, lights, 4);
		pixelArray[pixelIndexInImg].r = pixelColour.r * 255;
		pixelArray[pixelIndexInImg].b = pixelColour.b * 255;
		pixelArray[pixelIndexInImg].g = pixelColour.g * 255;
	}
	else
	{
		pixelArray[pixelIndexInImg].r = 0;
		pixelArray[pixelIndexInImg].g = 0;
		pixelArray[pixelIndexInImg].b = 0;
	}
}

bool traceNode(Node *node, Ray &ray, HitInfo &hitInfo, int hitSide)
{
	Ray nodeSpaceRay;
	bool status = false, status1 = false;

	nodeSpaceRay = node->ToNodeCoords(ray);  // transforming the camera ray to the node space.

	for (int i = 0; i < node->GetNumChild(); i++) {
		float z = hitInfo.z;
		if (status1 = traceNode(node->GetChild(i), nodeSpaceRay, hitInfo, hitSide))
		{
			if (z > hitInfo.z)
				node->FromNodeCoords(hitInfo);
		}
		status = status || status1;
	}

	if (node->GetNodeObj())
	{
		float z = hitInfo.z;
		if (node->GetNodeObj()->IntersectRay(nodeSpaceRay, hitInfo, hitSide))
		{
			if (z > hitInfo.z)
			{
				hitInfo.node = node;
				node->FromNodeCoords(hitInfo);
			}
			status = true;
		}
	}

	return status;
}

void StopRender()
{}
