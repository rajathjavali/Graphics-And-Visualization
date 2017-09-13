#pragma once
#include "scene.h"
#include "objects.h"
#include "tinyxml/tinyxml.h"

int LoadScene(const char *filename);
void LoadScene(TiXmlElement *element);
void LoadNode(Node *node, TiXmlElement *element, int level = 0);
void LoadTransform(Transformation *trans, TiXmlElement *element, int level);
void ReadVector(TiXmlElement *element, Point3 &v);
void ReadFloat(TiXmlElement *element, float  &f, const char *name = "value");