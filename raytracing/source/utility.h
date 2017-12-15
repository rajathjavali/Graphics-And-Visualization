#ifndef _UTILITY_H_
#define _UTILITY_H_

// Settings
#define Halton_Random_Sampling true //false uses random, true uses halton for camera ray position
#define GI_MONTE_CARLO_INTEGRATION 0
#define GI_PATH_TRACER 0
#define GAMMA_CORRECTION 1
#define RANDOM_SAMPLING 0

#define PHOTON_MAP_ENABLED 1
#define CAUSTIC_MAP_ENABLED 1
#define PHOTON_MAP_HAS_DIRECT_LIGHT true	
#define PHOTON_MAP_DIRECT_SAMPLING true

//scene_prj2.xml, simple_box_scene.xml, scene_prj3.xml, scene_prj4.xml, Cornell_Box_Scene.xml, example_box_tri_obj.xml, helicopter.xml, test_proj8.xml, warrior_scene_prj10.xml
#define IMG_NAME "final"
#define RESOURCE_NAME "resource\\final.xml"

// Constants
#define PI 3.141593
#define EPSILON_SMALL 1e-7

// Photon Map
#define PHOTONSCOUNT 1000000
#define CAUSTIC_PHOTONS_COUNT 10000
#define PHOTON_BOUNCE_COUNT 10
#define CAUSTIC_PHOTON_BOUNCE_COUNT 20

// Adaptive Sampling
#define MIN_SAMPLES 32	
#define MAX_SAMPLES 64
#define VARIANCE 0.001
#define ADAPTIVE true 

// Ray Differentials for Sampling Textures
#define RAY_DIFFERENTIAL true

// Reflection and Refraction Bounce Count
#define MAX_RAY_BOUNCE_COUNT 10

// MonteCarlo Global Illumination
#define GI_MIN_RAYS 1
#define GI_MAX_RAYS 5
#define MAX_GI_BOUNCE_COUNT 2

// Path Tracer
#define GI_MAX_DEPTH_PATH_TRACING 10 // For Path Bounces

Node rootNode;
Camera camera;
RenderImage renderImage;
MaterialList materials;
LightList lights;
ObjFileList objList;
TexturedColor background;
TexturedColor environment;
TextureList textureList;

void Init();
void Trace(int i, int j);
extern void ShowViewport();
extern int LoadScene(const char *filename);
void BeginRender();
void Render(int i, int n);
void StopRender();
bool traceNode(Node *node, DifRays &ray, HitInfo &hitInfo, int hitSide);
Color computeGlobalIllumination(const Point3 &p, const Point3 &N, int bounceCount);
Color pathTracer(const Point3 &pos, const Point3 &normal, int bounceCount);
void createPhotonMap();
void createCausticMap();

float totalIntensity;
bool stopRender = false;

float maxVal(Color c)
{
	return (c.b > c.g ? (c.b > c.r ? c.b : c.r) : (c.g > c.r ? c.g : c.r));
}

// Map 2D points to hemisphere
#define ex 0
inline Point3 mapSamplesToHemisphere(Point2 p)
{
	float cos_phi = cos(2 * M_PI * p.x);
	float sin_phi = sin(2 * M_PI * p.x);
	float cos_theta = pow(1.0f - p.y, 1.0f / (1.0f + ex));
	float sin_theta = sqrt(1.0f - cos_theta * cos_theta);

	float pu = sin_theta * cos_phi;
	float pv = sin_theta * sin_phi;
	float pw = cos_theta;

	return Point3(pu, pv, pw);
}


// ----------------------------------------------------------------------------- COMPUTE TOTAL INTENSITY ------------------------------------------------------------------------------------------------
void computeIntensity()
{
	totalIntensity = 0;
	for (int i = 0; i < lights.size(); i++)
	{
		Light *li = lights.at(i);

		if (li->IsPhotonSource())
		{
			totalIntensity += maxVal(li->GetPhotonIntensity());
		}
	}
}

// ----------------------------------------------------------------------------- GET RANDOM LIGHT SOURCE ------------------------------------------------------------------------------------------------
Light* getRandLight()
{
	float randVal = ((double)rand() / (RAND_MAX)) * totalIntensity, li8 = 0;
	for (int i = 0; i < lights.size(); i++)
	{
		Light *li = lights.at(i);

		if (li->IsPhotonSource())
		{
			li8 += maxVal(li->GetPhotonIntensity());
			if (randVal <= li8)
				return li;
		}
	}
	return NULL;
}

// ----------------------------------------------------------------------------- GENERATING PHOTON DIRECTION ------------------------------------------------------------------------------------------------
Ray PointLight::RandomPhoton() const
{
	Point3 dir, newPt, xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1);
	float normalTheta, normalPhi, x, y, z;

	normalTheta = ((double)rand() / (RAND_MAX)) * 2 * PI;
	normalPhi = ((double)rand() / (RAND_MAX)) * 4 * PI;
	x = sin(normalTheta) * cos(normalPhi);
	y = sin(normalTheta) * sin(normalPhi);
	z = cos(normalTheta);

	newPt = x * xAxis + y * yAxis + z * zAxis;
	dir = newPt.GetNormalized();
	Ray photon(position, dir);

	return photon;
}

bool MtlBlinn::RandomCausticPhotonBounce(Ray &r, Color &c, const HitInfo &hInfo) const
{
	float probDistribution = 0, probFactor = 0, li8 = 0;
	float specMax = maxVal(reflection.Sample(hInfo.uvw)), refMax = maxVal(refraction.GetColor());
	probDistribution += specMax + refMax;
	
	float randVal = ((double)rand() / (RAND_MAX)) * probDistribution;
	if (randVal < (li8 += specMax))
	{
		Point3 dir = -2 * hInfo.N * (hInfo.N.Dot(r.dir)) + r.dir;
		r.p = hInfo.p;
		r.dir = dir.GetNormalized();

		probFactor = (float)specMax / probDistribution;
		c = c / (float)probFactor;
	}
	// refraction
	else if (randVal < (li8 += refMax))
	{
		float ratio = 1.0f / ior;
		float cos_i, cos_t;
		float sin_2t;
		Point3 t;

		cos_i = fabs(r.dir.Dot(hInfo.N));
		sin_2t = ratio * ratio * (1 - cos_i * cos_i);
		HitInfo newHitInfo;

		if (r.dir.Dot(hInfo.N) > 0) // hit inside
		{
			ratio = ior;
			sin_2t = ratio * ratio * (1 - cos_i * cos_i);


			if (sin_2t > 1) // totally internal reflection
			{
				r.dir = (r.dir - 2 * hInfo.N * (hInfo.N.Dot(r.dir))).GetNormalized();
			}
			else
			{
				cos_t = sqrt(1 - sin_2t);

				t = ratio * r.dir - (ratio * cos_i - cos_t) * hInfo.N;
				t.Normalize();
				r.dir = t;
			}
		}
		else // hit outside
		{
			cos_t = sqrt(1 - sin_2t);
			t = ratio * r.dir + (ratio * cos_i - cos_t) * hInfo.N;
			t.Normalize();
			r.dir = t;
		}
		r.p = hInfo.p;
		probFactor = (float)refMax / probDistribution;
		c = c / probFactor;
	}
	return true;
}

bool MtlBlinn::RandomPhotonBounce(Ray &r, Color &c, const HitInfo &hInfo) const
{
	float probDistribution = 0, probFactor = 0;
	float diffMax = maxVal(diffuse.Sample(hInfo.uvw)), specMax = maxVal(reflection.Sample(hInfo.uvw)), refMax = maxVal(refraction.GetColor());
	probDistribution += diffMax + specMax + refMax;
	float absorbMax = fmaxf(0, 1.0f - diffMax - specMax - refMax);

	Point3 newDir;
	float randVal = ((double)rand() / (RAND_MAX)) * probDistribution, li8 = 0;
	// diffuse reflection
	if (randVal < (li8 += diffMax))
	{
		Point3 T, S;
		Point3 xAxis(1, 0, 0), yAxis(0, 1, 0), li8Point;
		T = hInfo.N ^ xAxis;

		if (T.Length() < EPSILON_SMALL)
		{
			T = hInfo.N ^ yAxis;
		}

		T.Normalize();
		S = T ^ hInfo.N;
		S.Normalize();
		Point3 sample3 = mapSamplesToHemisphere(Point2(((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX))));

		li8Point = sample3.x * T + sample3.y * S + sample3.z * hInfo.N;

		if (li8Point.Dot(hInfo.N) < 0)
		{
			li8Point = -li8Point;
		}

		r.p = hInfo.p;
		r.dir = li8Point.GetNormalized();

		probFactor = (float)diffMax / probDistribution;
		c = c * diffuse.Sample(hInfo.uvw) / (float)probFactor;
	}
	// specular (reflection)
	else if (randVal <= (li8 += specMax))
	{
		Point3 dir = -2 * hInfo.N * (hInfo.N.Dot(r.dir)) + r.dir;
		r.p = hInfo.p;
		r.dir = dir.GetNormalized();

		probFactor = (float)specMax / probDistribution;
		c = c * reflection.Sample(hInfo.uvw) / (float)probFactor;
	}
	// refraction
	else if (randVal <= (li8 += refMax))
	{
		float ratio = 1.0f / ior;
		float cos_i, cos_t;
		float sin_2t;
		Point3 t;

		cos_i = fabs(r.dir.Dot(hInfo.N));
		sin_2t = ratio * ratio * (1 - cos_i * cos_i);
		HitInfo newHitInfo;

		if (r.dir.Dot(hInfo.N) > 0) // hit inside
		{
			ratio = ior;
			sin_2t = ratio * ratio * (1 - cos_i * cos_i);


			if (sin_2t > 1) // totally internal reflection
			{
				r.dir = r.dir - 2 * hInfo.N * (hInfo.N.Dot(r.dir));
			}
			else
			{
				cos_t = sqrt(1 - sin_2t);

				t = ratio * r.dir - (ratio * cos_i - cos_t) * hInfo.N;
				t.Normalize();
				r.dir = t;
			}
		}
		else // hit outside
		{
			cos_t = sqrt(1 - sin_2t);
			t = ratio * r.dir + (ratio * cos_i - cos_t) * hInfo.N;
			t.Normalize();
			r.dir = t;
		}

		r.p = hInfo.p;
		probFactor = (float)refMax / probDistribution;
		c = c * refraction.Sample(hInfo.uvw) / probFactor;
	}
	// russian roulette; killing the photon
	else
	{
		return false;
	}	
	return true;
}

// ------------------------------------------------------------------ HALTON NORMAL FOR GLOSSY SURFACE ------------------------------------------

Point3 getHaltonNormal(float radius, HitInfo hInfo, int raySampleCount)
{
	Point3 newNormal;
	float x, y, z, xdist, ydist, zdist, dist;
	Point3 xAxis = Point3(1, 0, 0), yAxis = Point3(0, 1, 0), zAxis(0, 0, 1);
	Point3 sphPoint, sphCent = hInfo.p + hInfo.N.GetNormalized(),
		sphBound = sphCent - radius * xAxis - radius * yAxis + radius * zAxis;

	bool status = false;
	float sqRadius = radius * radius;
	do {
		status = false;
		x = Halton(raySampleCount, 2); // Halton sequence base 2
		y = Halton(raySampleCount, 3); // Halton sequence base 3
		z = Halton(raySampleCount, 5); // Halton sequence base 5
		xdist = x * radius * 2;
		ydist = y * radius * 2;
		zdist = z * radius * 2;
		Point3 newPoint = sphBound + xdist * xAxis + ydist * yAxis - zdist * zAxis;

		dist = pow(sphCent.x - newPoint.x, 2) + pow(sphCent.y - newPoint.y, 2) + pow(sphCent.z - newPoint.z, 2);
		if (dist <= sqRadius)
			status = true, newNormal = (newPoint - hInfo.p).GetNormalized();
		else
			raySampleCount++;
	} while (!status && raySampleCount < 1000);

	return newNormal;
}

// ----------------------------------------------------------------------------- LIGHT ILLUMINATE OVERRIDE ------------------------------------------------------------------------------------------------
Color GILight::Illuminate(const Point3 &p, const Point3 &N) const
{
	return computeGlobalIllumination(p, N, MAX_RAY_BOUNCE_COUNT);
}

Color PointLight::Illuminate(const Point3 &p, const Point3 &N) const
{
	if (size) {
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
}

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
	{
		return 0;
	}
	return 1;
}

// ----------------------------------------------------------------------------- STOP RENDERING ------------------------------------------------------------------------------------------------
void StopRender()
{
	stopRender = true;
}
#endif