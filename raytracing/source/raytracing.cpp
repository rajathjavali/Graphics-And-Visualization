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
#define IMG_NAME "prj3.jpg"
#define Z_IMG_NAME "prj3_z.jpg"
#define RESOURCE_NAME "resource\\scene_prj3.xml" //scene_prj2.xml, simple_box_scene.xml, scene_prj3.xml

Node rootNode;
Camera camera;
RenderImage renderImage;
MaterialList materials;
LightList lights;
Point3 u, v, startingPoint;
float *distanceBuffer;
Color24 *pixelArray;

extern void ShowViewport();
extern int LoadScene(const char *filename);
bool traceNode(Node *node, Ray &ray, HitInfo &hitInfo, int hitSide);
bool traceShadow(Node *node, Ray &ray, HitInfo &hInfo, int hitSide);
void Trace(int i, int j);

int main()
{
	LoadScene(RESOURCE_NAME);
	
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
	// now traversing through all the pixel centers in the screen
	tbb::parallel_for(0, camera.imgHeight, [&](int j) {
		for (int i = 0; i < camera.imgWidth; i++) {
			Trace(i, j);
		}
	});
	/*for(int j = 0; j< camera.imgHeight; j++)
		for (int i = 0; i < camera.imgWidth; i++)
			Trace(i, j);*/

	clock_t endTime = clock();
	double timeElapsed = static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
	printf("\nRender time is %f seconds\n", timeElapsed);
	
	renderImage.SaveImage(IMG_NAME);
	renderImage.ComputeZBufferImage();
	renderImage.SaveZImage(Z_IMG_NAME);
	
	printf("Enter a key to exit\n");
	while (getchar())
	{
		break;
	}

	return 1;
}

// Blinn shading
// H = L+V/|L+V|
// I0(V) = Summation of all lights { Ii*(N*Li)(kd + ks (N*Hi)^a)} + Iamb * kd;  
// where I - intensity, N- normal to the material = ray.dir(if sphere), kd -diffusal const, ks - specular const, a - glossinesss
Color MtlBlinn::Shade(const Ray &ray, const HitInfo &hInfo, const LightList &lights) const
{

	Color color;
	color.r = color.b = color.g = 0;
	Point3 Hi;
	Point3 surfaceNormal = hInfo.N.GetNormalized();
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
		Point3 viewDir = ray.dir.GetNormalized();
		Hi = (lDir + viewDir).GetNormalized(); // half vector
		color += liIntensity * fmax(0, surfaceNormal.Dot(lDir)) * (diffuse + specular *  pow(fmax(0,surfaceNormal.Dot(Hi)), glossiness));
		color.ClampMinMax();
	}
	return color;

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
			if(z > hitInfo.z)
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
		pixelColour = hit_info.node->GetMaterial()->Shade(ray, hit_info, lights);
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

float GenLight::Shadow(Ray ray, float t_max)
{
	HitInfo hitInfo;
	hitInfo.z = t_max;
	if (t_max == 1) {
		float dist = sqrt(ray.dir.Dot(ray.dir));
		hitInfo.z = dist;
	}
	ray.Normalize();
	if (traceNode(&rootNode, ray, hitInfo, HIT_FRONT))
		return 0;
	return 1;
}


bool traceShadow(Node *node, Ray &ray, HitInfo &hitInfo, int hitSide)
{
	Ray nodeSpaceRay;	
	bool status = false, status1 = false;

	nodeSpaceRay = node->ToNodeCoords(ray);  // transforming the camera ray to the node space.

	for (int i = 0; i < node->GetNumChild(); i++) {
		if (traceShadow(node->GetChild(i), nodeSpaceRay, hitInfo, hitSide))
		{
			float z = hitInfo.z;
			if (status1 = traceShadow(node->GetChild(i), nodeSpaceRay, hitInfo, hitSide))
				if (z > hitInfo.z)
					node->FromNodeCoords(hitInfo);
			status = status || status1;
		}
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
	return false; 
}

void BeginRender()
{}

void StopRender()
{}
