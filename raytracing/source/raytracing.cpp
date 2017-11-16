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
#include "utility.h"

#include <thread>
#include <atomic>
#include <time.h>
#include <iostream>
#include <math.h>

using namespace tbb;


//#define IMG_NAME "prj11Cornell.jpg"
//#define Z_IMG_NAME "prj11Cornell_z.jpg"
//#define SAMPLE_MAP "prj11SampleCornell.jpg"
//#define RESOURCE_NAME "resource\\CornellBox.xml"
//scene_prj2.xml, simple_box_scene.xml, scene_prj3.xml, scene_prj4.xml, Cornell_Box_Scene.xml, example_box_tri_obj.xml, helicopter.xml, test_proj8.xml, warrior_scene_prj10.xml
#define IMG_NAME "prj11teapot.jpg"
#define Z_IMG_NAME "prj11teapot_z.jpg"
#define SAMPLE_MAP "prj11Sampleteapot.jpg"
#define RESOURCE_NAME "resource\\SkylightTeapot.xml"

//#define IMG_NAME "prj8.jpg"
//#define Z_IMG_NAME "prj8_z.jpg"
//#define SAMPLE_MAP "prj8Sample.jpg"
//#define RESOURCE_NAME "resource\\scene_prj7.xml"

Node rootNode;
Camera camera;
RenderImage renderImage;
MaterialList materials;
LightList lights;
ObjFileList objList;
TexturedColor background;
TexturedColor environment;
TextureList textureList;

Point3 vectU, vectV, u, v, startingPoint, cameraBoxStart;
Point3 cameraPoints[5];
float *distanceBuffer;
Color24 *pixelArray;
float *numRay;
int haltonReflectionCount, haltonRefractionCount;
int giBounce;

extern void ShowViewport();
extern int LoadScene(const char *filename);
void BeginRender();
bool traceNode(Node *node, DifRays &ray, HitInfo &hitInfo, int hitSide);
void StopRender();

// ----------------------------------------------------------------------------- VARIABLES INITIALIZER ------------------------------------------------------------------------------------------------
void Init()
{
	vectU = camera.dir ^ camera.up; // using the right hand thumb rule and cross product of unit vectors to get the direction in +x direction
	vectV = camera.dir ^ vectU; // v vector in -y direction
	if(camera.dof)
	cameraBoxStart = camera.pos - camera.dof * vectU - camera.dof * vectV;

	int imgSize = renderImage.GetWidth() * renderImage.GetHeight();
	pixelArray = renderImage.GetPixels();
	numRay = new float[800 * 600];
	float heightOfImgPlane = 2 * tan((camera.fov / 2) * PI / 180.0f) * camera.focaldist;
	float pixelSize = heightOfImgPlane / camera.imgHeight;
	distanceBuffer = renderImage.GetZBuffer();
	
	// setting the magnitude of the vectors equivalent to pixel size	
	u = vectU * pixelSize;
	v = vectV * pixelSize;

	startingPoint = camera.pos + camera.dir * camera.focaldist
		- v * (camera.imgHeight / 2) // since vector v is in -y direction and we need to go in +y direction to get to the top corner
		- u * (camera.imgWidth / 2); // since vector u is in +x direction and we need to go in -x direction to get to the top corner
}


// ----------------------------------------------------------------------------- MAIN ------------------------------------------------------------------------------------------------
int main()
{
	LoadScene(RESOURCE_NAME);

	//GILight *li = new GILight();
	//lights.push_back(li);

	std::thread(ShowViewport).detach();
	
	printf("Enter a key to exit\n");
	while (getchar())
	{
		break;
	}

	return 1;
}


// ----------------------------------------------------------------------------- BEGIN RENDER ------------------------------------------------------------------------------------------------
void BeginRender()
{
	Init();
	clock_t startTime = clock();
	tbb::task_scheduler_init init(6);
	tbb::parallel_for(0, camera.imgHeight, [&](int j) {
		//for (int j = 0; j < camera.imgHeight; j++) {
			for (int i = 0; i < camera.imgWidth; i++) {
				//if(i == 212 && j == 120)
				Trace(i, j);
			}
		//}
	});
	clock_t endTime = clock();

	double timeElapsed = static_cast<double>(endTime - startTime) / CLOCKS_PER_SEC;
	printf("\nRender time is %f seconds\n", timeElapsed);

	renderImage.SaveImage(IMG_NAME);
	renderImage.ComputeZBufferImage();
	renderImage.SaveZImage(Z_IMG_NAME);
	
	if (ADAPTIVE)
	{
		uchar *numberRayImg = NULL;
		int size = 800 * 600;
		if (numberRayImg) delete[] numberRayImg;
		numberRayImg = new uchar[size];

		float zmin = MIN_SAMPLES, zmax = MAX_SAMPLES;
		for (int i = 0; i < size; i++) {
			if (numRay[i] == BIGFLOAT) numRay[i] = 0;
			else {
				float f = 1 - (zmax - numRay[i]) / (zmax - zmin);
				int c = int(f * 255);
				if (c < 0) c = 0;
				if (c > 255) c = 255;
				numberRayImg[i] = c;
			}
		}
		renderImage.SavePNG(SAMPLE_MAP, numberRayImg, 1);
	}
}

// ----------------------------------------------------------------------------- RAY TRACING FUNCTION ------------------------------------------------------------------------------------------------
void Trace(int i, int j) {
	//Halton(int index, int base);
	Point3 dircenter, dirx, diry, pixel_center_ij, subpixel, antialiasingRayDir;

	Color pixelColour, sqColor, variance, rayColor, avgColor;
	pixelColour.SetBlack();
	sqColor.SetBlack();
	variance.SetBlack();
	rayColor.SetBlack();
	avgColor.SetBlack();

	int pixelIndexInImg = j * camera.imgWidth + i;
	distanceBuffer[pixelIndexInImg] = BIGFLOAT;
	numRay[pixelIndexInImg] = BIGFLOAT;
	int countRay = 1, countCamP = 1;
	float valx = 0, valy = 0, camvalx = 0, camvaly = 0;
	float sqr = pow(camera.dof, 2);
	while (countRay <= MAX_SAMPLES) {
		DifRays rays;
		bool status = false;

		haltonReflectionCount = countRay, haltonRefractionCount = countRay;
		Point3 cameraPoint = camera.pos;
		if (camera.dof) {
			if (countRay != 1)
			{
				if (!Halton_Random_Sampling)
				{
					float cpdVal = ((double)rand() / (RAND_MAX));
					float r = sqrt(cpdVal) * camera.dof;
					float theta = ((double)rand() / (RAND_MAX)) * 2 * PI;
					camvalx = r * cos(theta);
					camvaly = r * sin(theta);

					cameraPoint = camera.pos + camvalx * vectU + camvaly * vectV;
				}
				else
				{
					do {
						status = false;
						camvalx = Halton(countCamP, 3); // Halton sequence base 5
						camvaly = Halton(countCamP, 4); // Halton sequence base 7
						float xdist = camvalx * camera.dof * 2;
						float ydist = camvaly * camera.dof * 2;
						Point3 newPoint = cameraBoxStart + xdist * vectU + ydist * vectV;

						float dist = pow(camera.pos.x - newPoint.x, 2) + pow(camera.pos.y - newPoint.y, 2);
						if (dist <= sqr)
							status = true, cameraPoint = newPoint;

						countCamP++;

					} while (!status && countCamP < 1000);
				}
			}
		}

		//picking subpixel position
		valx = Halton(countRay, 2); // Halton sequence base 2
		valy = Halton(countRay, 3); // Halton sequence base 3


		subpixel = startingPoint + (i + valx * 1.2) * u + (j + valy * 1.2) * v;
		antialiasingRayDir = subpixel - cameraPoint;

		if (/*countRay <= 8 &&*/ RAY_DIFFERENTIAL)
		{
			pixel_center_ij = startingPoint + (i + 0.5) * u + (j + 0.5) * v;
			dircenter = pixel_center_ij - cameraPoint;
			dirx = startingPoint + (i + 1) * u + (j + 0.5) * v - cameraPoint;
			diry = startingPoint + (i + 0.5) * u + j * v - cameraPoint;

			rays = DifRays(cameraPoint, antialiasingRayDir, dircenter, dirx, diry);
		}
		else
			rays = DifRays(cameraPoint, antialiasingRayDir);

		HitInfo hit_info;

		if (traceNode(&rootNode, rays, hit_info, HIT_FRONT_AND_BACK))
		{
			if(cameraPoint == camera.pos)
				distanceBuffer[pixelIndexInImg] = hit_info.z;
			giBounce = 1;
			rayColor = hit_info.node->GetMaterial()->Shade(rays, hit_info, lights, MAX_RAY_BOUNCE_COUNT);
		}
		else
		{
			Point3 textureCoord = Point3((float)i / camera.imgWidth, (float)j / camera.imgHeight, 0);
			rayColor = background.Sample(textureCoord);
		}

		sqColor += rayColor * rayColor;
		pixelColour += rayColor;
		avgColor = pixelColour / countRay;
		variance = (sqColor / countRay) - (avgColor * avgColor);
		
		
		if (ADAPTIVE) {
			if (countRay >= MIN_SAMPLES && variance.r < VARIANCE && variance.b < VARIANCE && variance.g < VARIANCE)
				break;
		}		
		countRay++;
	}
	pixelArray[pixelIndexInImg] = Color24(avgColor);
	renderImage.IncrementNumRenderPixel(1);
	numRay[pixelIndexInImg] = countRay - 1;

}


// ----------------------------------------------------------------------------- SCENE TRACING AND HIT FINDING FUNCTION ------------------------------------------------------------------------------------------------
bool traceNode(Node *node, DifRays &rays, HitInfo &hitInfo, int hitSide)
{
	DifRays nodeSpaceRay;
	bool status = false, status1 = false;

	nodeSpaceRay.ray = node->ToNodeCoords(rays.ray);  // transforming the camera ray to the node space.
	nodeSpaceRay.difcenter = node->ToNodeCoords(rays.difcenter);
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

// ---------------------------------------------------------------- Computing Global Illumination for the given Point and its normal ----------------------------------------------------------
Color computeGlobalIllumination(const Point3 &p, const Point3 &N, int bounceCount)
{
	DifRays ray;
	int count = 0;
	bool status = false;
	Color indirectColor(0, 0, 0);
	float cpdVal, normalR, normalTheta, normalPhi, x, y, z, shadowVal = 0;
	Point3 xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1), li8Point, newDir;
	float cosine = 0;
	int numOfRays = giBounce == 1 ? GI_MAX_RAYS : (giBounce <= 4 ? GI_MIN_RAYS : 0);
	for (int i = 0; i < numOfRays; i++) {
		count = 0, status = false;
		do {
			cpdVal = ((double)rand() / (RAND_MAX));
			normalR = sqrt(cpdVal) * 1;
			normalTheta = ((double)rand() / (RAND_MAX)) * 2 * PI;
			normalPhi = ((double)rand() / (RAND_MAX)) * 4 * PI;
			x = normalR * sin(normalTheta) * cos(normalPhi);
			y = normalR * sin(normalTheta) * sin(normalPhi);
			z = normalR * cos(normalTheta);

			li8Point = p + x * xAxis + y * yAxis + z * zAxis;
			newDir = (li8Point - p).GetNormalized();
			cosine = newDir.Dot(N);
			if (cosine >= 0)
				status = true;
			else
			{
				newDir = -newDir;
				cosine = newDir.Dot(N);
				if (cosine >= 0)
					status = true;
				else
					count++;
			}
		} while (!status && count < 100);


		ray = DifRays(p, newDir);
		//shadowVal = Shadow(ray, 1);

		HitInfo temp;
		if (traceNode(&rootNode, ray, temp, HIT_FRONT_AND_BACK))
		{
			giBounce++;
			Color x = temp.node->GetMaterial()->Shade(ray, temp, lights, bounceCount - 1);
			indirectColor += x * cosine;
			giBounce--;
		}
		else {
			indirectColor += environment.SampleEnvironment(ray.ray.dir);
		}
	}
	if (indirectColor != Color(0, 0, 0))
		indirectColor = indirectColor / numOfRays;
	return indirectColor;
}

// ----------------------------------------------------------------------------- BLINN SHADER ------------------------------------------------------------------------------------------------
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

	Point3 Hi, HiX, HiY, HiCent, surfaceNormal = hInfo.N.GetNormalized();
	Point3 viewDirCenter, viewDirX, viewDirY, viewDir = rays.ray.dir.GetNormalized();

	if (rays.status) {
		viewDirCenter = rays.difcenter.dir.GetNormalized();
		viewDirX = rays.difx.dir.GetNormalized();	
		viewDirY = rays.dify.dir.GetNormalized();
	}
	float cosi, sini, sint, cost;

	for (int i = 0; i < lights.size(); i++)
	{
		Light *li = lights.at(i);
		
		if (li->IsAmbient())
		{
			color += diffuse.Sample(hInfo.uvw) * li->Illuminate(hInfo.p, surfaceNormal);
			continue;
		}
		Color liIntensity(0, 0, 0);
		liIntensity = li->Illuminate(hInfo.p, surfaceNormal);
		
		
		Point3 lDir = li->Direction(hInfo.p).GetNormalized();


		Hi = -(lDir + viewDir).GetNormalized(); // half vector
		if (rays.status)
		{
			HiCent = -(lDir + viewDirCenter).GetNormalized();
			HiX = -(lDir + viewDirX).GetNormalized();
			HiY = -(lDir + viewDirY).GetNormalized();

			color += liIntensity * fmax(0, surfaceNormal.Dot(-lDir)) * (diffuse.Sample(hInfo.uvw, hInfo.duvw, true) + specular.Sample(hInfo.uvw, hInfo.duvw, true)
				*  ((pow(fmax(0, surfaceNormal.Dot(HiCent)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiX)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiY)), glossiness)) / 3));

		}
		else
			color += liIntensity * fmax(0, surfaceNormal.Dot(-lDir)) * (diffuse.Sample(hInfo.uvw) + specular.Sample(hInfo.uvw) *  pow(fmax(0, surfaceNormal.Dot(Hi)), glossiness));
	}
	//Global Illumination
	{
		if (giBounce <= 3 ) {
			Color giColor = computeGlobalIllumination(hInfo.p, hInfo.N, bounceCount);
			color += giColor/* * diffuse.Sample(hInfo.uvw)*/;
		}
	}

	if (refraction.GetColor() != Color(0, 0, 0))
	{
		Point3 newNormal = -surfaceNormal;
		if (refractionGlossiness)		
			newNormal = -getHaltonNormal(refractionGlossiness, hInfo, haltonRefractionCount);

		cosi = newNormal.Dot(viewDir);
		sini = sqrt(1 - fmin(pow(cosi, 2), 1));

		HitInfo temp;
		temp.z = BIGFLOAT;
		//ray inside the object and getting a total internal reflection
		if (!hInfo.front) {
			sint = (float)sini * ior;
			if (sint > 1)
			{
				Point3 rayDir = (-2 * newNormal * (newNormal.Dot(viewDir)) + viewDir).GetNormalized();
				DifRays totalReflectedRay;
				if (rays.status)
				{
					Point3 rayDirCent = (-2 * newNormal * (newNormal.Dot(viewDirCenter)) + viewDirCenter).GetNormalized();
					Point3 rayDirX = (-2 * newNormal * (newNormal.Dot(viewDirX)) + viewDirX).GetNormalized();
					Point3 rayDirY = (-2 * newNormal * (newNormal.Dot(viewDirY)) + viewDirY).GetNormalized();

					totalReflectedRay = DifRays(hInfo.p, rayDir, rayDirCent, rayDirX, rayDirY);
				}
				else
					totalReflectedRay = DifRays(hInfo.p, rayDir);
				if (traceNode(&rootNode, totalReflectedRay, temp, HIT_FRONT_AND_BACK))
				{
					absorptionCoeff.r = exp(-hInfo.z * absorption.r);
					absorptionCoeff.g = exp(-hInfo.z * absorption.g);
					absorptionCoeff.b = exp(-hInfo.z * absorption.b);
					refractedColor += absorptionCoeff * temp.node->GetMaterial()->Shade(totalReflectedRay, temp, lights, (bounceCount - 1));
				}
				else {
					if(!totalReflectedRay.status)
						refractedColor += environment.SampleEnvironment(totalReflectedRay.ray.dir);
					else
						refractedColor += (environment.SampleEnvironment(totalReflectedRay.difcenter.dir) + environment.SampleEnvironment(totalReflectedRay.difx.dir) + environment.SampleEnvironment(totalReflectedRay.dify.dir)) / 3;
				}
			}
			else
			{
				cost = sqrt(1 - fmin(pow(sint, 2), 1));
				Point3 S = (newNormal.Cross(newNormal.Cross(-viewDir))).GetNormalized();
				Point3 refractedDir = (-newNormal * cost + S * sint).GetNormalized();
				DifRays refractedRay;
				if (rays.status)
				{
					Point3 Scenter = (newNormal.Cross(newNormal.Cross(-viewDirCenter))).GetNormalized();
					Point3 refractedDirCent = (-newNormal * cost + Scenter * sint).GetNormalized();

					Point3 Sx = (newNormal.Cross(newNormal.Cross(-viewDirX))).GetNormalized();
					Point3 refractedDirX = (-newNormal * cost + Sx * sint).GetNormalized();

					Point3 Sy = (newNormal.Cross(newNormal.Cross(-viewDirY))).GetNormalized();
					Point3 refractedDirY = (-newNormal * cost + Sy * sint).GetNormalized();

					refractedRay = DifRays(hInfo.p, refractedDir, refractedDirCent, refractedDirX, refractedDirY);
				}
				else
					refractedRay = DifRays(hInfo.p, refractedDir);


				if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
				{
					if (temp.front)
						refractedColor += temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount - 1));
				}
				else {
					if (!refractedRay.status)
						refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
					else
						refractedColor += (environment.SampleEnvironment(refractedRay.difcenter.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;
				}
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
			Point3 S = (newNormal.Cross(newNormal.Cross(viewDir))).GetNormalized();
			Point3 refractedDir = (-1 * newNormal * cost + S * sint).GetNormalized();
			
			DifRays refractedRay;
			if (rays.status)
			{
				Point3 Scent = (newNormal.Cross(newNormal.Cross(viewDirCenter))).GetNormalized();
				Point3 refractedDirCent = (-1 * newNormal * cost + Scent * sint).GetNormalized();

				Point3 Sx = (newNormal.Cross(newNormal.Cross(viewDirX))).GetNormalized();
				Point3 refractedDirX = (-1 * newNormal * cost + Sx * sint).GetNormalized();


				Point3 Sy = (newNormal.Cross(newNormal.Cross(viewDirY))).GetNormalized();
				Point3 refractedDirY = (-1 * newNormal * cost + Sy * sint).GetNormalized();
				refractedRay = DifRays(hInfo.p, -refractedDir, -refractedDirCent, -refractedDirX, -refractedDirY);
			}
			else
				refractedRay = DifRays(hInfo.p, -refractedDir);


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
			else {
				if(!refractedRay.status)
					refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
				else
					refractedColor += (environment.SampleEnvironment(refractedRay.difcenter.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;
			}
		}
		color += refraction.GetColor() * refractedColor;
	}

	if (totalReflection != Color(0, 0, 0))
	{

		Point3 newNormal = surfaceNormal;
		if (reflectionGlossiness)
			newNormal = getHaltonNormal(reflectionGlossiness, hInfo, haltonReflectionCount);

		Point3 rayDir = -2 * newNormal * (newNormal.Dot(viewDir)) + viewDir;
		rayDir.Normalize();

		DifRays reflectedRay;
		if (rays.status)
		{
			Point3 rayDirCent = -2 * newNormal * (newNormal.Dot(viewDirCenter)) + viewDirCenter;
			rayDirCent.Normalize();

			Point3 rayDirX = -2 * newNormal * (newNormal.Dot(viewDirX)) + viewDirX;
			rayDirX.Normalize();

			Point3 rayDirY = -2 * newNormal * (newNormal.Dot(viewDirY)) + viewDirY;
			rayDirY.Normalize();

			reflectedRay = DifRays(hInfo.p, rayDir, rayDirCent, rayDirX, rayDirY);
		}
		else
			reflectedRay = DifRays(hInfo.p, rayDir);

		HitInfo temp;
		temp.z = BIGFLOAT;

		if (traceNode(&rootNode, reflectedRay, temp, HIT_FRONT))
		{
			if (hInfo.front)
				color += totalReflection * temp.node->GetMaterial()->Shade(reflectedRay, temp, lights, (bounceCount - 1));
		}
		else {
			if(!reflectedRay.status)
				color += totalReflection * environment.SampleEnvironment(reflectedRay.ray.dir);
			else
				color += totalReflection * (environment.SampleEnvironment(reflectedRay.difcenter.dir) + environment.SampleEnvironment(reflectedRay.difx.dir) + environment.SampleEnvironment(reflectedRay.dify.dir)) / 3;
		}
	}
	color.ClampMinMax();
	return color;

}

// ----------------------------------------------------------------------------- LIGHT ILLUMINATE OVERRIDE ------------------------------------------------------------------------------------------------
Color GILight::Illuminate(const Point3 &p, const Point3 &N) const
{
	return computeGlobalIllumination(p, N, 4);
}

Color PointLight::Illuminate(const Point3 &p, const Point3 &N) const
{
	if (size && SHADOW_SAMPLING) {
		Point3 xAxis = Point3(1, 0, 0), yAxis = Point3(0, 1, 0), zAxis(0, 0, 1);
		float x, y, z;
		int countRay = 0, MaxRays = 8, nonShadow = 0, shadow = 0;
		bool status = false, fullSample = false;
		float normalR, normalTheta, normalPhi, cpdVal;

		while (countRay <= MaxRays) {
			cpdVal = ((double)rand() / (RAND_MAX));
			normalR = sqrt(cpdVal) * size;
			normalTheta = ((double)rand() / (RAND_MAX)) * 2 * PI;
			normalPhi = ((double)rand() / (RAND_MAX)) * 4 * PI;
			x = normalR * sin(normalTheta) * cos(normalPhi);
			y = normalR * sin(normalTheta) * sin(normalPhi);
			z = normalR * cos(normalTheta);

			Point3 li8Point = position + x * xAxis + y * yAxis + z * zAxis;
			Point3 newDir = li8Point - p;

			float val = Shadow(DifRays(p, newDir), 1);

			if (val == 1)
				nonShadow++;
			else
				shadow++;

			if (countRay == MaxRays)
			{
				if (!fullSample)
				{
					if (nonShadow + shadow == MaxRays && nonShadow != MaxRays && shadow != MaxRays)
					{
						MaxRays = 64;
						continue;
					}
				}
				return (float)nonShadow / MaxRays * intensity;
			}
			countRay++;
		}
	}
	
	return Shadow(DifRays(p, position - p), 1) * intensity;
};


// ----------------------------------------------------------------------------- SHADOW FINDING FUNCTION ------------------------------------------------------------------------------------------------
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

// ----------------------------------------------------------------------------- STOP RENDERING ------------------------------------------------------------------------------------------------
void StopRender()
{}
