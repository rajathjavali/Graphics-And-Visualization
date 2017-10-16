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
#define IMG_NAME "prj7_sampling.jpg"
#define Z_IMG_NAME "prj7_z.jpg"
#define RESOURCE_NAME "resource\\scene_prj7.xml" 
//scene_prj2.xml, simple_box_scene.xml, scene_prj3.xml, scene_prj4.xml, Cornell_Box_Scene.xml, example_box_tri_obj.xml, helicopter.xml

Node rootNode;
Camera camera;
RenderImage renderImage;
MaterialList materials;
LightList lights;
ObjFileList objList;
TexturedColor background;
TexturedColor environment;
TextureList textureList;

Point3 u, v, startingPoint;
float *distanceBuffer;
Color24 *pixelArray;

extern void ShowViewport();
extern int LoadScene(const char *filename);
void BeginRender();
void Init();
void Trace(int i, int j);
bool traceNode(Node *node, DifRays &ray, HitInfo &hitInfo, int hitSide);
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

int main()
{
	LoadScene(RESOURCE_NAME);

	std::thread *th = new std::thread(ShowViewport);
	
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
	Point3 dirx = startingPoint + (i + 1) * u + (j + 0.5) * v - camera.pos;
	Point3 diry = startingPoint + (i + 0.5) * u + j * v - camera.pos;

	Color pixelColour;
	pixelColour.SetBlack();

	DifRays rays(camera.pos, dir, dirx, diry);

	int pixelIndexInImg = j * camera.imgWidth + i;
	distanceBuffer[pixelIndexInImg] = BIGFLOAT;
	HitInfo hit_info;

	if (traceNode(&rootNode, rays, hit_info, HIT_FRONT))
	{
		distanceBuffer[pixelIndexInImg] = hit_info.z;
		pixelColour = hit_info.node->GetMaterial()->Shade(rays, hit_info, lights, 4);
		pixelArray[pixelIndexInImg].r = pixelColour.r * 255;
		pixelArray[pixelIndexInImg].b = pixelColour.b * 255;
		pixelArray[pixelIndexInImg].g = pixelColour.g * 255;
	}
	else
	{
		float heightOfImgPlane = 2 * tan((camera.fov / 2) * PI / 180.0f) * CAM_TO_IMG_DIST;
		float pixelSize = heightOfImgPlane / camera.imgHeight;
		float widthOfImgPlane = pixelSize * camera.imgWidth;
		Point3 textureCoord = Point3((float)i/camera.imgWidth, (float)j/camera.imgHeight, 0);
		//textureCoord.Normalize();
		Color backgnd = background.Sample(textureCoord);
		pixelArray[pixelIndexInImg].r = backgnd.r * 255;
		pixelArray[pixelIndexInImg].g = backgnd.g * 255;
		pixelArray[pixelIndexInImg].b = backgnd.b * 255;
	}
}

bool traceNode(Node *node, DifRays &rays, HitInfo &hitInfo, int hitSide)
{
	DifRays nodeSpaceRay;
	bool status = false, status1 = false;

	nodeSpaceRay.ray = node->ToNodeCoords(rays.ray);  // transforming the camera ray to the node space.
	nodeSpaceRay.difx = node->ToNodeCoords(rays.difx);
	nodeSpaceRay.dify = node->ToNodeCoords(rays.dify);

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

// Blinn shading
// H = L+V/|L+V|
// I0(V) = Summation of all lights { Ii*(N*Li)(kd + ks (N*Hi)^a)} + Iamb * kd;  
// where I - intensity, N- normal to the material = ray.dir(if sphere), kd -diffusal const, ks - specular const, a - glossinesss
Color MtlBlinn::Shade(const DifRays &rays, const HitInfo &hInfo, const LightList &lights, int bounceCount) const
{
	Color black = Color(0, 0, 0);
	if (bounceCount == 0)
		return black;

	if (!hInfo.front && refraction.GetColor() == Color(0, 0, 0))
		return black;

	Color color(0, 0, 0),
		absorptionCoeff(1, 1, 1),
		reflectedColor(0, 0, 0),
		refractedColor(0, 0, 0),
		totalReflection = reflection.GetColor();

	Point3 Hi, HiX, HiY, surfaceNormal = hInfo.N.GetNormalized();
	Point3 viewDir = rays.ray.dir.GetNormalized();
	Point3 viewDirX = rays.difx.dir.GetNormalized();
	Point3 viewDirY = rays.dify.dir.GetNormalized();

	float cosi, sini, sint, cost;

	for (int i = 0; i < lights.size(); i++)
	{
		Light *li = lights.at(i);
		Color liIntensity = li->Illuminate(hInfo.p, surfaceNormal);
		if (li->IsAmbient())
		{
			color += diffuse.Sample(hInfo.uvw) * liIntensity;
			continue;
		}

		Point3 lDir = li->Direction(hInfo.p).GetNormalized();


		Hi = (lDir + viewDir).GetNormalized(); // half vector
		HiX = (lDir + viewDirX).GetNormalized();
		HiY = (lDir + viewDirY).GetNormalized();

		//color += liIntensity * fmax(0, surfaceNormal.Dot(lDir)) * (diffuse.Sample(hInfo.uvw) + specular.Sample(hInfo.uvw) *  pow(fmax(0, surfaceNormal.Dot(Hi)), glossiness));
		color += liIntensity * fmax(0, surfaceNormal.Dot(lDir)) * (diffuse.Sample(hInfo.uvw, hInfo.duvw, true) + specular.Sample(hInfo.uvw, hInfo.duvw, true) 
							 *  ((pow(fmax(0, surfaceNormal.Dot(HiX)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiX)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiY)), glossiness)) / 3));
	}
	if (refraction.GetColor() != Color(0, 0, 0))
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
				Point3 rayDirX = (-2 * surfaceNormal * (surfaceNormal.Dot(viewDirX)) + viewDirX).GetNormalized();
				Point3 rayDirY = (-2 * surfaceNormal * (surfaceNormal.Dot(viewDirY)) + viewDirY).GetNormalized();

				DifRays totalReflectedRay(hInfo.p, rayDir, rayDirX, rayDirY);
				if (traceNode(&rootNode, totalReflectedRay, temp, HIT_FRONT_AND_BACK))
				{
					absorptionCoeff.r = exp(-hInfo.z * absorption.r);
					absorptionCoeff.g = exp(-hInfo.z * absorption.g);
					absorptionCoeff.b = exp(-hInfo.z * absorption.b);
					refractedColor += absorptionCoeff * temp.node->GetMaterial()->Shade(totalReflectedRay, temp, lights, (bounceCount - 1));
				}
				else
					//refractedColor += environment.SampleEnvironment(totalReflectedRay.ray.dir);
					refractedColor += (environment.SampleEnvironment(totalReflectedRay.ray.dir) + environment.SampleEnvironment(totalReflectedRay.difx.dir) + environment.SampleEnvironment(totalReflectedRay.dify.dir)) / 3;
			}
			else
			{
				cost = sqrt(1 - fmin(pow(sint, 2), 1));
				Point3 S = (surfaceNormal.Cross(surfaceNormal.Cross(-viewDir))).GetNormalized();
				Point3 refractedDir = (-surfaceNormal * cost + S * sint).GetNormalized();

				Point3 Sx = (surfaceNormal.Cross(surfaceNormal.Cross(-viewDirX))).GetNormalized();
				Point3 refractedDirX = (-surfaceNormal * cost + Sx * sint).GetNormalized();

				Point3 Sy = (surfaceNormal.Cross(surfaceNormal.Cross(-viewDirY))).GetNormalized();
				Point3 refractedDirY = (-surfaceNormal * cost + Sy * sint).GetNormalized();

				DifRays refractedRay(hInfo.p, refractedDir, refractedDirX, refractedDirY);

				if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
				{
					if (temp.front)
						refractedColor += temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount - 1));
				}
				else
					//refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
					refractedColor += (environment.SampleEnvironment(refractedRay.ray.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;
			}
		}
		else
		{
			sint = (float)sini / ior;

			float R0 = (float)(pow((1 - ior), 2)) / pow((1 + ior), 2);
			float reflectedPortion = R0 + (1 - R0) * pow((1 - cosi), 5);
			float refractedPortion = 1 - reflectedPortion;
			totalReflection += reflectedPortion * refraction.GetColor();

			cost = sqrt(1 - fmin(pow(sint, 2), 1));
			Point3 S = (surfaceNormal.Cross(surfaceNormal.Cross(viewDir))).GetNormalized();
			Point3 refractedDir = (-1 * surfaceNormal * cost + S * sint).GetNormalized();
			

			Point3 Sx = (surfaceNormal.Cross(surfaceNormal.Cross(viewDirX))).GetNormalized();
			Point3 refractedDirX = (-1 * surfaceNormal * cost + Sx * sint).GetNormalized();


			Point3 Sy = (surfaceNormal.Cross(surfaceNormal.Cross(viewDirY))).GetNormalized();
			Point3 refractedDirY = (-1 * surfaceNormal * cost + Sy * sint).GetNormalized();
			
			DifRays refractedRay(hInfo.p, -refractedDir, -refractedDirX, -refractedDirY);

			if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
			{
				if (!temp.front)
				{
					absorptionCoeff.r = exp(-temp.z * absorption.r);
					absorptionCoeff.g = exp(-temp.z * absorption.g);
					absorptionCoeff.b = exp(-temp.z * absorption.b);
					refractedColor += absorptionCoeff * refractedPortion * temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount - 1));
				}
			}
			else
				//refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
				refractedColor += (environment.SampleEnvironment(refractedRay.ray.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;

		}
		color += refraction.GetColor() * refractedColor;
	}

	if (totalReflection != Color(0, 0, 0))
	{
		Point3 rayDir = -2 * surfaceNormal * (surfaceNormal.Dot(viewDir)) + viewDir;
		rayDir.Normalize();

		Point3 rayDirX = -2 * surfaceNormal * (surfaceNormal.Dot(viewDirX)) + viewDirX;
		rayDirX.Normalize();

		Point3 rayDirY = -2 * surfaceNormal * (surfaceNormal.Dot(viewDirY)) + viewDirY;
		rayDirY.Normalize();
		
		DifRays reflectedRay(hInfo.p, rayDir, rayDirX, rayDirY);

		HitInfo temp;
		temp.z = BIGFLOAT;

		if (traceNode(&rootNode, reflectedRay, temp, HIT_FRONT))
			if (hInfo.front)
				color += totalReflection * temp.node->GetMaterial()->Shade(reflectedRay, temp, lights, (bounceCount - 1));
			else
				//color += environment.SampleEnvironment(reflectedRay.ray.dir);
				color += (environment.SampleEnvironment(reflectedRay.ray.dir) + environment.SampleEnvironment(reflectedRay.difx.dir) + environment.SampleEnvironment(reflectedRay.dify.dir)) / 3;
	}
	color.ClampMinMax();
	return color;

}

float GenLight::Shadow(DifRays rays, float t_max)
{
	HitInfo hitInfo;
	hitInfo.z = t_max;
	if (t_max == 1) {
		float dist = sqrt(rays.ray.dir.Dot(rays.ray.dir));
		hitInfo.z = dist;
	}
	rays.ray.Normalize();
	if (traceNode(&rootNode, rays, hitInfo, HIT_FRONT_AND_BACK))
		return 0;
	return 1;
}

void StopRender()
{}
