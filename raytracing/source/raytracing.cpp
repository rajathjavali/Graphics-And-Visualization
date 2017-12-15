/*
*  raytracing.cpp
*  Created on: Aug 25, 2017
*      Author: RajathJavali
*/
// raytracing.cpp : Defines the entry point for the console application.
#include "scene.h"
#include "objects.h"
#include "materials.h"
#include "lights.h"
#include "tbb/tbb.h"
#include "tbb/task_scheduler_init.h"
#include "utility.h"
#include "tracer.h"
#include "cyPhotonMap.h"

#include <thread>
#include <atomic>
#include <time.h>
#include <iostream>
#include <math.h>

using namespace tbb;

Point3 vectU, vectV, u, v, startingPoint, cameraBoxStart;
int haltonReflectionCount, haltonRefractionCount, giBounce, pathTracerDepth;
cyPhotonMap *photonMap, *causticMap;
std::string imgfilename;
// ----------------------------------------------------------------------------- VARIABLES INITIALIZER ------------------------------------------------------------------------------------------------
void Init()
{
	imgfilename = IMG_NAME;
	vectU = camera.dir ^ camera.up; // using the right hand thumb rule and cross product of unit vectors to get the direction in +x direction
	vectV = camera.dir ^ vectU; // v vector in -y direction
	if(camera.dof)
	cameraBoxStart = camera.pos - camera.dof * vectU - camera.dof * vectV;

	int imgSize = renderImage.GetWidth() * renderImage.GetHeight();
	float heightOfImgPlane = 2 * tan((camera.fov / 2) * PI / 180.0f) * camera.focaldist;
	float pixelSize = heightOfImgPlane / camera.imgHeight;
	
	// setting the magnitude of the vectors equivalent to pixel size	
	u = vectU * pixelSize;
	v = vectV * pixelSize;

	startingPoint = camera.pos + camera.dir * camera.focaldist
		- v * (camera.imgHeight / 2) // since vector v is in -y direction and we need to go in +y direction to get to the top corner
		- u * (camera.imgWidth / 2); // since vector u is in +x direction and we need to go in -x direction to get to the top corner

	computeIntensity();
	#if PHOTON_MAP_ENABLED
		createPhotonMap();
	#endif // PHOTON_MAP_ENABLED

	#if CAUSTIC_MAP_ENABLED
		createCausticMap();
	#endif // CAUSTIC_MAP_ENABLED


}

void createPhotonMap()
{
	photonMap = new cyPhotonMap();
	photonMap->Resize(PHOTONSCOUNT);
	int photonCount = 0;
	int photonNum = (unsigned int)photonMap->NumPhotons();

	while (photonNum < PHOTONSCOUNT) {
		Light* Li = NULL;
		Li = getRandLight();
		Color LiI = Li->GetPhotonIntensity();
		float prob = ((float)maxVal(LiI)) / (float)totalIntensity;
		LiI /= (float)prob;
		Ray ray = Li->RandomPhoton();
		int bounce = 1;
		bool status = false;
		while (bounce < PHOTON_BOUNCE_COUNT)
		{
			HitInfo hInfo;
			if (traceNode(&rootNode, DifRays(ray), hInfo, HIT_FRONT_AND_BACK))
			{
				#if PHOTON_MAP_HAS_DIRECT_LIGHT
				if (bounce >= 1/* && hInfo.front*/ /*&& hInfo.node->GetMaterial()->IsPhotonSurface()*/)
				#else
				if (bounce > 1 && hInfo.front /*&& hInfo.node->GetMaterial()->IsPhotonSurface()*/)
				#endif	
				{
					if (isnan(LiI.r) || isnan(LiI.r) || isnan(LiI.r))
						break;
					if (!photonMap->AddPhoton(hInfo.p, ray.dir, LiI))
						break;
					status = true;
				}
				if (!hInfo.node->GetMaterial()->RandomPhotonBounce(ray, LiI, hInfo))
					break;	
				bounce++;
			}
			else
				break;
		}
		if (status)
			photonCount++;
		photonNum = photonMap->NumPhotons();
	};

	photonMap->PrepareForIrradianceEstimation();
	photonMap->ScalePhotonPowers((float)10 / (photonCount));
	FILE *fp = fopen("photonmap.dat", "wb");
	fwrite(photonMap->GetPhotons(), sizeof(cyPhotonMap::Photon), photonMap->NumPhotons(), fp);
	fclose(fp);
}

void createCausticMap()
{
	causticMap = new cyPhotonMap();
	causticMap->Resize(CAUSTIC_PHOTONS_COUNT);
	int photonCount = 0;
	int photonNum = (unsigned int)causticMap->NumPhotons();
	tbb::task_scheduler_init init(4);
	//tbb::parallel_for(0, CAUSTIC_PHOTONS_COUNT-1, [&](int j) {
	while (photonNum < CAUSTIC_PHOTONS_COUNT) {
		Light* Li = NULL;
		Li = getRandLight();
		Color LiI = Li->GetPhotonIntensity();
		float prob = ((float)maxVal(LiI)) / (float)totalIntensity;
		LiI /= (float)prob;
		Ray ray = Li->RandomPhoton();
		Ray initRay = ray;
		int bounce = 1;
		bool specularHits = false;
		while (bounce < CAUSTIC_PHOTON_BOUNCE_COUNT)
		{
			HitInfo hInfo;
			if (traceNode(&rootNode, DifRays(ray), hInfo, HIT_FRONT))
			{
				if (hInfo.node->GetMaterial()->isSpecular())
				{
					specularHits = true;
					hInfo.node->GetMaterial()->RandomCausticPhotonBounce(ray, LiI, hInfo);
				}
				else
				{
					if (specularHits)
					{
						specularHits = false;
						LiI /= (initRay.p - hInfo.p).LengthSquared();
						if (isnan(LiI.r) || isnan(LiI.g) || isnan(LiI.b))
							break;

						causticMap->AddPhoton(hInfo.p, ray.dir, LiI);
						break;
					}
					else
						break;
				}
				bounce++;
			}
			else
				break;
		}
		//});
		photonNum = causticMap->NumPhotons();
	}
	causticMap->ScalePhotonPowers((float)1 / photonNum);
	causticMap->PrepareForIrradianceEstimation();

	FILE *fp = fopen("causticmap.dat", "wb");
	fwrite(causticMap->GetPhotons(), sizeof(cyPhotonMap::Photon), causticMap->NumPhotons(), fp);
	fclose(fp);
}

// ----------------------------------------------------------------------------- MAIN ------------------------------------------------------------------------------------------------
int main()
{
	
	LoadScene(RESOURCE_NAME);
	Init();

	//std::thread(ShowViewport).detach();
	ShowViewport();
	printf("Enter a key to exit\n");
	while (getchar())
	{
		break;
	}

	return 1;
}


// ----------------------------------------------------------------------------- BEGIN RENDER ------------------------------------------------------------------------------------------------
std::thread **RenderThreads = NULL;
int NThreads = std::thread::hardware_concurrency();
void BeginRender() {
	if (NThreads <= 1) NThreads = 1;
	if (RenderThreads) {
		for (int i = 0; i < NThreads; ++i) {
			if (RenderThreads[i]) {
				if (RenderThreads[i]->joinable()) {
					RenderThreads[i]->join();
				}
				delete RenderThreads[i];
			}
		}
	}
	delete[] RenderThreads;

	renderImage.ResetNumRenderedPixels();

	stopRender = false;
	RenderThreads = new std::thread*[NThreads];
	for (int i = 0; i < NThreads; ++i) {
		RenderThreads[i] = new std::thread(Render, i, NThreads);
	}
}

void Render(int i, int n) {
	int begin = float(camera.imgHeight) * float(i) / n;
	int end = float(camera.imgHeight) * float(i + 1) / n;

	for (int ii = begin; ii < end; ++ii) {
		if (stopRender) {
			return;
		}
		for (int jj = 0; jj < camera.imgWidth; ++jj) {
			Trace(jj, ii);
		}
	}

}

void saveImg()
{
	renderImage.SaveImage(std::string(imgfilename + ".jpg").c_str());
	renderImage.ComputeZBufferImage();
	renderImage.SaveZImage(std::string(imgfilename + "Z.jpg").c_str());
	renderImage.ComputeSampleCountImage();
	renderImage.SaveSampleCountImage(std::string(imgfilename + "_sample.jpg").c_str());
}

// ----------------------------------------------------------------------------- RAY TRACING FUNCTION ------------------------------------------------------------------------------------------------
void Trace(int i, int j) {
	Point3 dircenter, dirx, diry, pixel_center_ij, subpixel, antialiasingRayDir;

	Color pixelColour, sqColor, variance, rayColor, avgColor;
	pixelColour.SetBlack();
	sqColor.SetBlack();
	variance.SetBlack();
	rayColor.SetBlack();
	avgColor.SetBlack();

	int pixelIndexInImg = j * camera.imgWidth + i;
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
	
				#if !Halton_Random_Sampling
					float cpdVal = ((double)rand() / (RAND_MAX));
					float r = sqrt(cpdVal) * camera.dof;
					float theta = ((double)rand() / (RAND_MAX)) * 2 * PI;
					camvalx = r * cos(theta);
					camvaly = r * sin(theta);

					cameraPoint = camera.pos + camvalx * vectU + camvaly * vectV;
				 // !Halton_Random_Sampling
				#else
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
				#endif		
			}
		}

		//picking subpixel position
		valx = Halton(countRay, 2); // Halton sequence base 2
		valy = Halton(countRay, 3); // Halton sequence base 3


		subpixel = startingPoint + (i + valx * 1.2) * u + (j + valy * 1.2) * v;
		antialiasingRayDir = subpixel - cameraPoint;

		if (countRay <= 16 && RAY_DIFFERENTIAL)
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
				renderImage.GetZBuffer()[pixelIndexInImg] = hit_info.z;
			giBounce = 1;
			pathTracerDepth = 0;
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
	#if GAMMA_CORRECTION
		float gammaInv = (float)1 / 2.2;
		avgColor.r = pow(avgColor.r, gammaInv);
		avgColor.g = pow(avgColor.g, gammaInv);
		avgColor.b = pow(avgColor.b, gammaInv);
	#endif
	renderImage.GetPixels()[pixelIndexInImg] = Color24(avgColor);
	renderImage.GetSampleCount()[pixelIndexInImg] = countRay - 1;
	renderImage.IncrementNumRenderPixel(1);

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
	Color indirectColor(0, 0, 0);
	
	Point3 xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1), li8Point, newDir;
	int numOfRays = giBounce == 1 ? GI_MAX_RAYS : (giBounce <= MAX_GI_BOUNCE_COUNT ? GI_MIN_RAYS : 0);
	if (numOfRays == 0)
		return Color(0, 0, 0);

	#if RANDOM_SAMPLING
		float cpdVal, normalR = 1, normalTheta, normalPhi, x, y, z, shadowVal = 0, cosine = 0;
		int count = 0;
		bool status = false;
	#else
		Point3 T, S;
		float randTheta, cosWeightedTheta, randPhi;

		if (fabs(N.Dot(xAxis)) != 1)
			T = (N.Cross(xAxis)).GetNormalized();
		else
			T = (N.Cross(yAxis)).GetNormalized();

		S = (T.Cross(N)).GetNormalized();
	#endif

	for (int i = 0; i < numOfRays; i++) {
	
	#if RANDOM_SAMPLING
		count = 0, status = false;
		do {
			/*cpdVal = ((double)rand() / (RAND_MAX));
			normalR = sqrt(cpdVal) * 1;*/
			normalTheta = ((double)rand() / (RAND_MAX)) * 2 * PI;
			normalPhi = ((double)rand() / (RAND_MAX)) * 4 * PI;
			x = normalR * sin(normalTheta) * cos(normalPhi);
			y = normalR * sin(normalTheta) * sin(normalPhi);
			z = normalR * cos(normalTheta);

			li8Point = x * xAxis + y * yAxis + z * zAxis;
			newDir = li8Point.GetNormalized();
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

		HitInfo temp;
		if (traceNode(&rootNode, ray, temp, HIT_FRONT_AND_BACK))
		{
			giBounce++;
			indirectColor += temp.node->GetMaterial()->Shade(ray, temp, lights, bounceCount - 1) * cosine;
			giBounce--;
		}
		else {
			Color color = environment.SampleEnvironment(ray.ray.dir);
			if (isnan(color.r)|| isnan(color.g)|| isnan(color.b))
				color = Color(0, 0, 0);
			indirectColor +=  color * cosine;
		}

	#else
		float sintheta, costheta;
		randTheta = ((double)rand() / (RAND_MAX));

		cosWeightedTheta = acos(1 - 2 * randTheta) / 2;
		randPhi = ((double)rand() / (RAND_MAX)) * 2 * PI;

		li8Point = p + N * cos(cosWeightedTheta) + T * sin(cosWeightedTheta) * cos(randPhi) + S * sin(cosWeightedTheta) * sin(randPhi);
		newDir = (li8Point - p).GetNormalized();

		ray = DifRays(p, newDir);

		HitInfo temp;
		if (traceNode(&rootNode, ray, temp, HIT_FRONT_AND_BACK))
		{
			giBounce++;
			indirectColor += temp.node->GetMaterial()->Shade(ray, temp, lights, bounceCount - 1);
			giBounce--;
		}
		else { 
			indirectColor += environment.SampleEnvironment(ray.ray.dir);
		}
	#endif
	}
	if (indirectColor != Color(0, 0, 0))
		indirectColor = indirectColor / numOfRays;
	return indirectColor;
}

Color photonMapGI(const Point3 &p, const Point3 &N)
{
	DifRays ray;
	Color color(0, 0, 0);

	Point3 xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1), li8Point, newDir;
	int numOfRays = 20;

	Point3 T, S;
	float randTheta, cosWeightedTheta, randPhi;

	if (fabs(N.Dot(xAxis)) != 1)
		T = (N.Cross(xAxis)).GetNormalized();
	else
		T = (N.Cross(yAxis)).GetNormalized();

	S = (T.Cross(N)).GetNormalized();

	for (int i = 0; i < numOfRays; i++) {

		float sintheta, costheta;
		randTheta = ((double)rand() / (RAND_MAX));

		cosWeightedTheta = acos(1 - 2 * randTheta) / 2;
		randPhi = ((double)rand() / (RAND_MAX)) * 2 * PI;

		li8Point = N * cos(cosWeightedTheta) + T * sin(cosWeightedTheta) * cos(randPhi) + S * sin(cosWeightedTheta) * sin(randPhi);
		newDir = li8Point.GetNormalized();

		ray = DifRays(p, newDir);

		HitInfo temp;
		if (traceNode(&rootNode, ray, temp, HIT_FRONT_AND_BACK))
		{
			Color c; Point3 dir;
			photonMap->EstimateIrradiance<50>(c, dir, 0.5, temp.p, &temp.N, 1.0F, cy::PhotonMap::FILTER_TYPE_CONSTANT);
			c /= cos(cosWeightedTheta);
			if (isnan(c.r) || isnan(c.g) || isnan(c.b))
				c = Color(0, 0, 0);
			color += c;
		}
		else {
			Color enviColor = environment.SampleEnvironment(newDir) / (float)fabs(newDir.Dot(N));
			enviColor /= cos(cosWeightedTheta);
			if (isnan(enviColor.r) || isnan(enviColor.g) || isnan(enviColor.b))
				enviColor = Color(0, 0, 0);
			color += enviColor;
		}
	}
	if (color != Color(0, 0, 0))
		color /= numOfRays;

	return color;
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

	/*if (!hInfo.front && refraction.GetColor() == Color(0, 0, 0))
		return black;*/

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

	Color liIntensity(0, 0, 0);
	Point3 lDir(0, 0, 0);


	Color liIntensityCaustic(0, 0, 0), liIntensityPhoton(0, 0, 0);
	#if PHOTON_MAP_ENABLED
	{
		for (int i = 0; i < lights.size(); i++)
		{
			Light *li = lights.at(i);

			if (li->IsAmbient())
			{
				color += diffuse.Sample(hInfo.uvw) * li->Illuminate(hInfo.p, surfaceNormal);
				continue;
			}
			#if PHOTON_MAP_HAS_DIRECT_LIGHT
			if (!li->IsPhotonSource()) 
			#endif		
			{
				liIntensity.SetBlack();
				liIntensity = li->Illuminate(hInfo.p, surfaceNormal);

				lDir = li->Direction(hInfo.p).GetNormalized();

				Hi = -(lDir + viewDir).GetNormalized(); // half vector
				if (rays.status)
				{
					HiCent = -(lDir + viewDirCenter).GetNormalized();
					HiX = -(lDir + viewDirX).GetNormalized();
					HiY = -(lDir + viewDirY).GetNormalized();

					color += liIntensity * fmax(0, surfaceNormal.Dot(-lDir)) * (diffuse.Sample(hInfo.uvw, hInfo.duvw, true) + specular.Sample(hInfo.uvw, hInfo.duvw, true)
						*  ((pow(fmax(0, surfaceNormal.Dot(HiCent)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiX)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiY)), glossiness)) / 3)) / li->totalDist(hInfo.p);

				}
				else
					color += liIntensity * surfaceNormal.Dot(-lDir) * (diffuse.Sample(hInfo.uvw) + specular.Sample(hInfo.uvw) *  pow(surfaceNormal.Dot(Hi), glossiness)) / li->totalDist(hInfo.p);
			}
		}
		#if !PHOTON_MAP_DIRECT_SAMPLING
		causticMap->EstimateIrradiance<50>(liIntensityCaustic, lDir, 0.5, hInfo.p, &hInfo.N, 1.0F, cyPhotonMap::FILTER_TYPE_CONSTANT);
		liIntensityPhoton = photonMapGI(hInfo.p, hInfo.N);
		#else
		photonMap->EstimateIrradiance<1000>(liIntensityPhoton, lDir, 2, hInfo.p, &hInfo.N, 1.0F, cyPhotonMap::FILTER_TYPE_CONSTANT);
		//causticMap->EstimateIrradiance<50>(liIntensityCaustic, lDir, 0.5, hInfo.p, &hInfo.N, 1.0F, cyPhotonMap::FILTER_TYPE_CONSTANT);
		#endif
		
		liIntensity = liIntensityPhoton + liIntensityCaustic;
		color += liIntensity * diffuse.Sample(hInfo.uvw);
	}
	#else
	{
		for (int i = 0; i < lights.size(); i++)
		{
			Light *li = lights.at(i);
		
			if (li->IsAmbient())
			{
				color += diffuse.Sample(hInfo.uvw) * li->Illuminate(hInfo.p, surfaceNormal);
				continue;
			}
			liIntensity.SetBlack();
			liIntensity = li->Illuminate(hInfo.p, surfaceNormal);
		
		
			lDir = li->Direction(hInfo.p).GetNormalized();


			Hi = -(lDir + viewDir).GetNormalized(); // half vector
			if (rays.status)
			{
				HiCent = -(lDir + viewDirCenter).GetNormalized();
				HiX = -(lDir + viewDirX).GetNormalized();
				HiY = -(lDir + viewDirY).GetNormalized();

				color += liIntensity * fmax(0, surfaceNormal.Dot(-lDir)) * (diffuse.Sample(hInfo.uvw, hInfo.duvw, true) + specular.Sample(hInfo.uvw, hInfo.duvw, true)
					*  ((pow(fmax(0, surfaceNormal.Dot(HiCent)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiX)), glossiness) + pow(fmax(0, surfaceNormal.Dot(HiY)), glossiness)) / 3))/* / li->totalDist(hInfo.p)*/;

			}
			else
				color += liIntensity * fmax(0, surfaceNormal.Dot(-lDir)) * (diffuse.Sample(hInfo.uvw) + specular.Sample(hInfo.uvw) *  pow(fmax(0, surfaceNormal.Dot(Hi)), glossiness))/* / li->totalDist(hInfo.p)*/;
		}

		//Global Illumination
		#if GI_MONTE_CARLO_INTEGRATION
		{
			Color giColor = computeGlobalIllumination(hInfo.p, hInfo.N, bounceCount);
			color += giColor * diffuse.Sample(hInfo.uvw);
		}
		#endif // GI_MONTE_CARLO_INTEGRATION
		#if GI_PATH_TRACER
		{
			color += pathTracer(hInfo.p, hInfo.N, bounceCount) * diffuse.Sample(hInfo.uvw);
		}
		#endif // GI_PATH_TRACER
	}
	#endif // PHOTON_MAP_ENABLED

	if (refraction.GetColor().Gray() > 0)
	{
		float ratio = 1.0f / ior;
		Point3 newNormal = surfaceNormal;
		if (refractionGlossiness)		
			newNormal = getHaltonNormal(refractionGlossiness, hInfo, haltonRefractionCount);
		float sin2t, cost, cosi;
		Point3 t;
		cosi = fabs(viewDir.Dot(newNormal));
		sin2t = ratio * ratio * (1 - cosi * cosi);
		HitInfo temp;
		temp.z = BIGFLOAT;
		if (viewDir.Dot(newNormal) > 0) // hit inside
		{
			ratio = ior;
			sin2t = ratio * ratio * (1 - cosi * cosi);

			if (sin2t > 1) // totally internal reflection
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
					if (!totalReflectedRay.status)
						refractedColor += environment.SampleEnvironment(totalReflectedRay.ray.dir);
					else
						refractedColor += (environment.SampleEnvironment(totalReflectedRay.difcenter.dir) + environment.SampleEnvironment(totalReflectedRay.difx.dir) + environment.SampleEnvironment(totalReflectedRay.dify.dir)) / 3;
				}
			}
			else
			{
				cost = sqrt(1 - sin2t);
				float R0 = (float)(pow((1 - ior), 2)) / pow((1 + ior), 2);
				float reflectedPortion = R0 + (1 - R0) * pow((1 - cosi), 5);
				float refractedPortion = 1 - reflectedPortion;
				totalReflection += reflectedPortion * refraction.GetColor();

				t = ratio * viewDir - (ratio * cosi - cost) * newNormal;
				t.Normalize();

				DifRays refractedRay;
				if (rays.status)
				{
					Point3 refractedDirCent = (ratio * viewDirCenter - (ratio * cosi - cost) * newNormal).GetNormalized();

					Point3 refractedDirX = (ratio * viewDirX - (ratio * cosi - cost) * newNormal).GetNormalized();

					Point3 refractedDirY = (ratio * viewDirY - (ratio * cosi - cost) * newNormal).GetNormalized();

					refractedRay = DifRays(hInfo.p, t, refractedDirCent, refractedDirX, refractedDirY);
				}
				else
					refractedRay = DifRays(hInfo.p, t);


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
			float R0 = (float)(pow((1 - ior), 2)) / pow((1 + ior), 2);
			float reflectedPortion = R0 + (1 - R0) * pow((1 - cosi), 5);
			float refractedPortion = 1 - reflectedPortion;
			totalReflection += reflectedPortion * refraction.GetColor();

			cost = sqrt(1 - sin2t);
			Point3 refractedDir = (ratio * viewDir + (ratio * cosi - cost) * newNormal).GetNormalized();
				
			DifRays refractedRay;
			if (rays.status)
			{
				Point3 refractedDirCent = (ratio * viewDir + (ratio * cosi - cost) * newNormal).GetNormalized();

				Point3 refractedDirX = (ratio * viewDir + (ratio * cosi - cost) * newNormal).GetNormalized();

				Point3 refractedDirY = (ratio * viewDir + (ratio * cosi - cost) * newNormal).GetNormalized();
				refractedRay = DifRays(hInfo.p, refractedDir, refractedDirCent, refractedDirX, refractedDirY);
			}
			else
				refractedRay = DifRays(hInfo.p, refractedDir);


			if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
			{

					absorptionCoeff.r = exp(-temp.z * absorption.r);
					absorptionCoeff.g = exp(-temp.z * absorption.g);
					absorptionCoeff.b = exp(-temp.z * absorption.b);
					refractedColor += absorptionCoeff * refractedPortion * temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount - 1));
			}
			else {
				if(!refractedRay.status)
					refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
				else
					refractedColor += (environment.SampleEnvironment(refractedRay.difcenter.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;
			}
		// Method 2
		//float cosi, sini, sint, cost;
		////newNormal = -newNormal;
		//cosi = newNormal.Dot(viewDir);
		//sini = sqrt(1 - fmin(pow(cosi, 2), 1));

		//HitInfo temp;
		//temp.z = BIGFLOAT;
		////ray inside the object and getting a total internal reflection
		//if (!hInfo.front) {
		//	sint = (float)sini * ior;
		//	if (sint > 1)
		//	{
		//		Point3 rayDir = (-2 * newNormal * (newNormal.Dot(viewDir)) + viewDir).GetNormalized();
		//		DifRays totalReflectedRay;
		//		if (rays.status)
		//		{
		//			Point3 rayDirCent = (-2 * newNormal * (newNormal.Dot(viewDirCenter)) + viewDirCenter).GetNormalized();
		//			Point3 rayDirX = (-2 * newNormal * (newNormal.Dot(viewDirX)) + viewDirX).GetNormalized();
		//			Point3 rayDirY = (-2 * newNormal * (newNormal.Dot(viewDirY)) + viewDirY).GetNormalized();

		//			totalReflectedRay = DifRays(hInfo.p, rayDir, rayDirCent, rayDirX, rayDirY);
		//		}
		//		else
		//			totalReflectedRay = DifRays(hInfo.p, rayDir);
		//		if (traceNode(&rootNode, totalReflectedRay, temp, HIT_FRONT_AND_BACK))
		//		{
		//			absorptionCoeff.r = exp(-hInfo.z * absorption.r);
		//			absorptionCoeff.g = exp(-hInfo.z * absorption.g);
		//			absorptionCoeff.b = exp(-hInfo.z * absorption.b);
		//			refractedColor += absorptionCoeff * temp.node->GetMaterial()->Shade(totalReflectedRay, temp, lights, (bounceCount - 1));
		//		}
		//		else {
		//			if(!totalReflectedRay.status)
		//				refractedColor += environment.SampleEnvironment(totalReflectedRay.ray.dir);
		//			else
		//				refractedColor += (environment.SampleEnvironment(totalReflectedRay.difcenter.dir) + environment.SampleEnvironment(totalReflectedRay.difx.dir) + environment.SampleEnvironment(totalReflectedRay.dify.dir)) / 3;
		//		}
		//	}
		//	else
		//	{
		//		cost = sqrt(1 - fmin(pow(sint, 2), 1));
		//		Point3 S = (newNormal.Cross(newNormal.Cross(-viewDir))).GetNormalized();
		//		Point3 refractedDir = (-newNormal * cost + S * sint).GetNormalized();
		//		DifRays refractedRay;
		//		if (rays.status)
		//		{
		//			Point3 Scenter = (newNormal.Cross(newNormal.Cross(-viewDirCenter))).GetNormalized();
		//			Point3 refractedDirCent = (-newNormal * cost + Scenter * sint).GetNormalized();

		//			Point3 Sx = (newNormal.Cross(newNormal.Cross(-viewDirX))).GetNormalized();
		//			Point3 refractedDirX = (-newNormal * cost + Sx * sint).GetNormalized();

		//			Point3 Sy = (newNormal.Cross(newNormal.Cross(-viewDirY))).GetNormalized();
		//			Point3 refractedDirY = (-newNormal * cost + Sy * sint).GetNormalized();

		//			refractedRay = DifRays(hInfo.p, refractedDir, refractedDirCent, refractedDirX, refractedDirY);
		//		}
		//		else
		//			refractedRay = DifRays(hInfo.p, refractedDir);


		//		if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
		//		{
		//			if (temp.front)
		//				refractedColor += temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount - 1));
		//		}
		//		else {
		//			if (!refractedRay.status)
		//				refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
		//			else
		//				refractedColor += (environment.SampleEnvironment(refractedRay.difcenter.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;
		//		}
		//	}
		//}
		//else
		//{
		//	sint = (float)sini / ior;

		//	float R0 = (float)(pow((1 - ior), 2)) / pow((1 + ior), 2);
		//	float reflectedPortion = R0 + (1 - R0) * pow((1 - cosi), 5);
		//	float refractedPortion = 1 - reflectedPortion;
		//	totalReflection += reflectedPortion * refraction.GetColor();

		//	cost = sqrt(1 - fmin(pow(sint, 2), 1));
		//	Point3 S = (newNormal.Cross(newNormal.Cross(viewDir))).GetNormalized();
		//	Point3 refractedDir = (-1 * newNormal * cost + S * sint).GetNormalized();
		//	
		//	DifRays refractedRay;
		//	if (rays.status)
		//	{
		//		Point3 Scent = (newNormal.Cross(newNormal.Cross(viewDirCenter))).GetNormalized();
		//		Point3 refractedDirCent = (-1 * newNormal * cost + Scent * sint).GetNormalized();

		//		Point3 Sx = (newNormal.Cross(newNormal.Cross(viewDirX))).GetNormalized();
		//		Point3 refractedDirX = (-1 * newNormal * cost + Sx * sint).GetNormalized();


		//		Point3 Sy = (newNormal.Cross(newNormal.Cross(viewDirY))).GetNormalized();
		//		Point3 refractedDirY = (-1 * newNormal * cost + Sy * sint).GetNormalized();
		//		refractedRay = DifRays(hInfo.p, -refractedDir, -refractedDirCent, -refractedDirX, -refractedDirY);
		//	}
		//	else
		//		refractedRay = DifRays(hInfo.p, -refractedDir);


		//	if (traceNode(&rootNode, refractedRay, temp, HIT_FRONT_AND_BACK))
		//	{
		//		if (!temp.front)
		//		{
		//			absorptionCoeff.r = exp(-temp.z * absorption.r);
		//			absorptionCoeff.g = exp(-temp.z * absorption.g);
		//			absorptionCoeff.b = exp(-temp.z * absorption.b);
		//			refractedColor += absorptionCoeff * refractedPortion * temp.node->GetMaterial()->Shade(refractedRay, temp, lights, (bounceCount - 1));
		//		}
		//	}
		//	else {
		//		if(!refractedRay.status)
		//			refractedColor += environment.SampleEnvironment(refractedRay.ray.dir);
		//		else
		//			refractedColor += (environment.SampleEnvironment(refractedRay.difcenter.dir) + environment.SampleEnvironment(refractedRay.difx.dir) + environment.SampleEnvironment(refractedRay.dify.dir)) / 3;
		//	}
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
			if (!reflectedRay.status)
				color += totalReflection * environment.SampleEnvironment(reflectedRay.ray.dir);
			else
				color += totalReflection * (environment.SampleEnvironment(reflectedRay.difcenter.dir) + environment.SampleEnvironment(reflectedRay.difx.dir) + environment.SampleEnvironment(reflectedRay.dify.dir)) / 3;
		}
	}
	color.ClampMinMax();
	return color;

}


Color pathTracer(const Point3 &pos, const Point3 &normal, int bounceCount)
{
	DifRays ray;
	Color indirectColor(0, 0, 0);
	float randTheta, cosWeightedTheta, randPhi, cosine = 0;

	Point3 xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1), li8Point, newDir;
	if (pathTracerDepth > GI_MAX_DEPTH_PATH_TRACING)
		return Color(0, 0, 0);

	#if RANDOM_SAMPLING
		float normalTheta, normalPhi, x, y, z;

		normalTheta = ((double)rand() / (RAND_MAX)) * 2 * PI;
		normalPhi = ((double)rand() / (RAND_MAX)) * 4 * PI;
		x = 1 * sin(normalTheta) * cos(normalPhi);
		y = 1 * sin(normalTheta) * sin(normalPhi);
		z = 1 * cos(normalTheta);

		li8Point = x * xAxis + y * yAxis + z * zAxis;
		newDir = li8Point.GetNormalized();
		cosine = newDir.Dot(normal);

		if (cosine < 0)
		{
			newDir = -newDir;
			cosine = newDir.Dot(normal);
		}
	#else
		Point3 T, S;
		cosine = 1;
		if (fabs(normal.Dot(xAxis)) != 1)
			T = (normal.Cross(xAxis)).GetNormalized();
		else
			T = (normal.Cross(yAxis)).GetNormalized();

		S = (T.Cross(normal)).GetNormalized();

		float sintheta, costheta;
		randTheta = ((double)rand() / (RAND_MAX));

		cosWeightedTheta = acos(1 - 2 * randTheta) / 2;
		randPhi = ((double)rand() / (RAND_MAX)) * 2 * PI;

		li8Point = normal * cos(cosWeightedTheta) + T * sin(cosWeightedTheta) * cos(randPhi) + S * sin(cosWeightedTheta) * sin(randPhi);
		newDir = li8Point.GetNormalized();
		cosine = fabs(newDir.Dot(normal));
	#endif // RANRANDOM_SAMPLING

	ray = DifRays(pos, newDir);

	HitInfo temp;
	if (traceNode(&rootNode, ray, temp, HIT_FRONT_AND_BACK))
	{
		pathTracerDepth++;
		indirectColor = temp.node->GetMaterial()->Shade(ray, temp, lights, bounceCount-1);
	}
	else {
		indirectColor = environment.SampleEnvironment(ray.ray.dir);
		if (isnan(indirectColor.r)|| isnan(indirectColor.g)|| isnan(indirectColor.b))
			return Color(0, 0, 0);
	}
	
	return indirectColor * cosine;
}