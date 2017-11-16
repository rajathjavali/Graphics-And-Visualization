#ifndef _UTILITY_H_
#define _UTILITY_H_



#define IMG_NAME "prj11Cornell.jpg"
#define Z_IMG_NAME "prj11Cornell_z.jpg"
#define SAMPLE_MAP "prj11SampleCornell.jpg"
#define RESOURCE_NAME "resource\\CornellBox.xml"
//scene_prj2.xml, simple_box_scene.xml, scene_prj3.xml, scene_prj4.xml, Cornell_Box_Scene.xml, example_box_tri_obj.xml, helicopter.xml, test_proj8.xml, warrior_scene_prj10.xml
//#define IMG_NAME "prj11teapot.jpg"
//#define Z_IMG_NAME "prj11teapot_z.jpg"
//#define SAMPLE_MAP "prj11Sampleteapot.jpg"
//#define RESOURCE_NAME "resource\\SkylightTeapot.xml"

//#define IMG_NAME "prj8.jpg"
//#define Z_IMG_NAME "prj8_z.jpg"
//#define SAMPLE_MAP "prj8Sample.jpg"
//#define RESOURCE_NAME "resource\\scene_prj7.xml"


#define PI 3.141592653589793
#define MIN_SAMPLES 8
#define MAX_SAMPLES 64
#define RAY_DIFFERENTIAL false
#define VARIANCE 0.001
#define ADAPTIVE true
#define Halton_Random_Sampling true //false uses random, true uses halton for camera ray position
#define SHADOW_SAMPLING true
#define MAX_RAY_BOUNCE_COUNT 16
#define MAX_GI_BOUNCE_COUNT 1
#define GI_MAX_RAYS 10
#define GI_MIN_RAYS 1


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
void StopRender();
bool traceNode(Node *node, DifRays &ray, HitInfo &hitInfo, int hitSide);
Color computeGlobalIllumination(const Point3 &p, const Point3 &N, int bounceCount);

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

#endif