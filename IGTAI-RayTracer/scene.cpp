#include "scene.h"
#include "scene_types.h"
#include <string.h>
#include <algorithm>
// 
// Object * initCone(vec3 vecteur, point3 centre, float radius, float h, Material mat) {
//     Object *ret;
//     ret = (Object *)malloc(sizeof(Object));
//     ret->geom.type = CONE;
//     ret->geom.cone.vecteur = vecteur;
//     ret->geom.cone.centre = centre;
//     ret->geom.cone.radius = radius;
//     ret->geom.cone.h = h;
//     memcpy(&(ret->mat), &mat, sizeof(Material));
//     return ret;
// }

Object *initCercle(float r, point3 centre, vec3 normal, Material mat) {
    Object *ret;
    ret = (Object *)malloc(sizeof(Object));
    ret->geom.type = CERCLE;
    ret->geom.cercle.centre = centre;
    ret->geom.cercle.normal = normal;
    ret->geom.cercle.radius = r;
    memcpy(&(ret->mat), &mat, sizeof(Material));
    return ret;
}
Object *initCylindre(float r, float h, vec3 centre, int orientation, Material mat) {
    Object *ret;
    ret = (Object *)malloc(sizeof(Object));
    ret->geom.type = CYLINDRE;
    ret->geom.cylindre.radius = r;
    if (h == 0) {// cylindre infini
      ret->geom.cylindre.height = HMAX;
    }else { //cylindre fini
      ret->geom.cylindre.height = h;
    }
    ret->geom.cylindre.centre = centre;
    ret->geom.cylindre.orientation = orientation;
    memcpy(&(ret->mat), &mat, sizeof(Material));
    return ret;
}

Object *initTriangle(point3 pointA, point3 pointB, point3 pointC, Material mat) {
    Object *ret;
    ret = (Object *)malloc(sizeof(Object));
    ret->geom.type = TRIANGLE;
    ret->geom.triangle.pointA = pointA;
    ret->geom.triangle.pointB = pointB;
    ret->geom.triangle.pointC = pointC;
    memcpy(&(ret->mat), &mat, sizeof(Material));
    return ret;
}

Object *initSphere(point3 center, float radius, Material mat) {
    Object *ret;
    ret = (Object *)malloc(sizeof(Object));
    ret->geom.type = SPHERE;
    ret->geom.sphere.center = center;
    ret->geom.sphere.radius = radius;
    memcpy(&(ret->mat), &mat, sizeof(Material));
    return ret;
}

Object *initPlane(vec3 normal, float d, Material mat) {
    Object *ret;
    ret = (Object *)malloc(sizeof(Object));
    ret->geom.type = PLANE;
    ret->geom.plane.normal = normalize(normal);
    ret->geom.plane.dist = d;
    memcpy(&(ret->mat), &mat, sizeof(Material));
    return ret;
}

void freeObject(Object *obj) {
    free(obj);
}

Light *initLight(point3 position, color3 color) {
    Light *light = (Light*)malloc(sizeof(Light));
    light->position = position;
    light->color = color;
    return light;
}

void freeLight(Light *light) {
    free(light);
}

Scene * initScene() {
    return new Scene;
}

void freeScene(Scene *scene) {
    std::for_each(scene->objects.begin(), scene->objects.end(), freeObject);
    std::for_each(scene->lights.begin(), scene->lights.end(), freeLight);
    delete scene;
}

void setCamera(Scene *scene, point3 position, point3 at, vec3 up, float fov, float aspect) {
    scene->cam.fov = fov;
    scene->cam.aspect = aspect;
    scene->cam.position = position;
    scene->cam.zdir = normalize(at-position);
    scene->cam.xdir = normalize(cross(up, scene->cam.zdir));
    scene->cam.ydir = normalize(cross(scene->cam.zdir, scene->cam.xdir));
    scene->cam.center = 1.f / tanf ((scene->cam.fov * M_PI / 180.f) * 0.5f) * scene->cam.zdir;
}

void addObject(Scene *scene, Object *obj) {
    scene->objects.push_back(obj);
}

void addLight(Scene *scene, Light *light) {
    scene->lights.push_back(light);
}

void setSkyColor(Scene *scene, color3 c) {
    scene->skyColor = c;
}
