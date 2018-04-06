
#include "raytracer.h"
#include "scene_types.h"
#include "ray.h"
#include "image.h"
#include "kdtree.h"
#include <stdio.h>
#include <math.h>
// thibault.lejemble@irit.fr
#define NbRayon 2
#define NbRebond 6
color3 anti_aliasing(Scene *scene, KdTree *tree, int n, vec3 dx, vec3 dy, vec3 centre, int i, int j);
bool solver(float a, float b, float c, float*t);
/// acne_eps is a small constant used to prevent acne when computing intersection
//  or boucing (add this amount to the position before casting a new ray !
const float acne_eps = 1e-4;
const float pi = 3.1415926f;
const float EPSILON = 0.0000001;

// bool intersectCone(Ray *ray, Intersection *intersection, Object *obj) {
//     vec3 O = ray->orig;
//     vec3 D = ray->dir;
//     float cost = cos(pi/2);
//     float cos2 = cost*cost;
//
//     point3 P; //point d'intersection
//     vec3 v = obj->geom.cone.vecteur;
//     point3 C = obj->geom.cone.centre; //C
//     float r = obj->geom.cone.radius; //r
//     vec3 X = O-C;
//     // float h = obj->geom.cone.h;
//     float m =
//
//     float a =
//     float b =
//     float c =
//     float t;
//     float d = b*b-4*a*c;
//
//     if (d < 0) return false; //delatat degatuve
//
//     if (d == 0) { //bcp de cas
//       // if(cosa == dot(v, ht)) return false;
//       t = -b/2*a;
//     }else {//delata > 0
//       t = (-b-sqrt(d))/(2*a);
//       if (t < 0) {
//         t = (-b+sqrt(d))/2*a;
//       }
//     }
//
//     if (ray->tmin < t && ray->tmax > t) { // on est dans l'intervalle
//       P = rayAt(*ray, t);
//       float R = P.y;
//       if (R <= C.y || R >= C.y + h) return false;
//       R =sqrt((P.x-C.x)*(P.x-C.x)+(P.z-C.z)*(P.z-C.z));
//       intersection->position = P; //ray->orig+t*obj->geom.plane.dist;
//       intersection->mat = &(obj->mat);
//       intersection->normal = point3(P.x-C.x,0,P.z-C.z);
//       ray->tmax = t; //mise à jour du tmax pour evité l'effet de bord
//       return true;
//     }
//     return false;
// }

bool intersectCercle(Ray *ray, Intersection *intersection, Object *obj) {
  Geometry g = obj->geom;
  point3 p;
  float r = g.cercle.radius;
  vec3 n = g.cercle.normal;
  point3 c = g.cercle.centre;
  point3 o = ray->orig;
  vec3 d = ray->dir;

  float distOrigin = -dot(n,c);
  float t;
  float DdotN = dot(d,n);
  if(DdotN==0.f)
    return false;


  t = (-(dot(o,n)+distOrigin))/DdotN;

  if(t < ray->tmin || t> ray->tmax)
    return false;
  p=rayAt(*ray,t);
  if(distance(p,c)>r)
    return false;

  ray->tmax=t;
  ray->depth++;

  intersection->mat=&obj->mat;
  intersection->position = p;
  intersection->normal = (DdotN<0.f?1.f:-1.f)*n;
  return true;

}
bool intersectCylindreY(Ray *ray, Intersection *intersection, Object *cylindre) {
  point3 center = cylindre->geom.cylindre.centre;
  float radius = cylindre->geom.cylindre.radius;
  float height = cylindre->geom.cylindre.height;

  point3 o = ray->orig;
  point3 p;
  vec3 d = ray->dir;
  float A = o.x - center.x;
  float B = o.z - center.z;

  float a = (d.x * d.x) + (d.z * d.z);
  float b = (2*A*d.x) + (2*B*d.z);
  float c = (A*A) + (B*B) - radius*radius;

  float t;


  if(!solver(a, b, c, &t)) return false;

  if(t > ray->tmin && t < ray->tmax) {

    p = rayAt(*ray,t);
    float r = p.y;

    if (r < center.y || r > center.y + height) return false;
    vec3 n;
    n = vec3 (p.x-center.x, 0, p.z-center.z);
    ray->tmax=t;
    ray->depth++;
    intersection->normal=normalize(n);
    intersection->mat=&(cylindre->mat);
    intersection->position = rayAt(*ray,t);
    return true;
  }
  return false;
}
bool intersectCylindreX(Ray *ray, Intersection *intersection, Object *cylindre) {
  point3 center = cylindre->geom.cylindre.centre;
  float radius = cylindre->geom.cylindre.radius;
  float height = cylindre->geom.cylindre.height;

  point3 o = ray->orig;
  point3 p;
  vec3 d = ray->dir;
  float A = o.y - center.y;
  float B = o.z - center.z;

  float a = (d.y * d.y) + (d.z * d.z);
  float b = (2*A*d.y) + (2*B*d.z);
  float c = (A*A) + (B*B) - radius*radius;

  float t;


  if(!solver(a, b, c, &t)) return false;

  if(t > ray->tmin && t < ray->tmax) {

    p = rayAt(*ray,t);
    float r = p.x;

    if (r < center.x || r > center.x + height) return false;
    vec3 n;
    n = vec3 (0, p.y-center.z, p.z-center.z);
    ray->tmax=t;
    ray->depth++;
    intersection->normal=normalize(n);
    intersection->mat=&(cylindre->mat);
    intersection->position = rayAt(*ray,t);
    return true;
  }
  return false;
}

bool intersectCylindreZ(Ray *ray, Intersection *intersection, Object *cylindre) {
  point3 center = cylindre->geom.cylindre.centre;
  float radius = cylindre->geom.cylindre.radius;
  float height = cylindre->geom.cylindre.height;

  point3 o = ray->orig;
  point3 p;
  vec3 d = ray->dir;
  float A = o.x - center.x;
  float B = o.y - center.y;

  float a = (d.x * d.x) + (d.y * d.y);
  float b = (2*A*d.x) + (2*B*d.y);
  float c = (A*A) + (B*B) - radius*radius;

  float t;


  if(!solver(a, b, c, &t)) return false;

  if(t > ray->tmin && t < ray->tmax) {

    p = rayAt(*ray,t);
    float r = p.z;

    if (r < center.z || r > center.z + height) return false;
    vec3 n;
    n = vec3 (p.x-center.x, p.y-center.y, 0);
    ray->tmax=t;
    ray->depth++;
    intersection->normal=normalize(n);
    intersection->mat=&(cylindre->mat);
    intersection->position = rayAt(*ray,t);
    return true;
  }
  return false;
}

//intersection en fonction de l'oriantation
bool intersectCylindre(Ray *ray, Intersection *intersection, Object *cylindre) {
  switch (cylindre->geom.cylindre.orientation) {
    case 0:
      return intersectCylindreY(ray, intersection, cylindre);
      break;
    case 1:
      return intersectCylindreX(ray, intersection, cylindre);
      break;
    case 2:
      return intersectCylindreZ(ray, intersection, cylindre);
      break;
    default:
      return intersectCylindreY(ray, intersection, cylindre);
      break;
  }
}


bool intersectTriangle(Ray *ray, Intersection *intersection, Object *obj) {
  //calcule de Discriminent
  point3 A = obj->geom.triangle.pointA;
  point3 B = obj->geom.triangle.pointB;
  point3 C = obj->geom.triangle.pointC;
  vec3 E1 = A - C;
  vec3 E2 = B - C;
  vec3 T = ray->orig - C;
  vec3 n = normalize(cross(E2, E1));

  float d_n = dot(ray->dir, n);

  if (d_n == 0) return false;
  if (d_n >= 0) {
    n = -1.f*n;
  }
  vec3 p = cross(ray->dir, E2);
  vec3 q = cross(T, E1);

  float det = dot(p, E1);
  // float det = -dot(cross(C, B), A);
  if (det == 0) return false;
  float invDet = 1/det;
  float u = invDet * dot(p, T);
  if (u < 0) return false;

  float v = invDet*dot(q, ray->dir);

  if (v < 0 || (u+v > 1)) return false;

  // if (u+v > 1) return false;

  float t = invDet*dot(q, E2);
  if (t > ray->tmin && t < ray->tmax) { // il y'a une intersection

    intersection->normal = n;
    intersection->position = rayAt(*ray, t);
    intersection->mat = &(obj->mat);
    ray->tmax = t;
    return true;
  }
  return false;
}

bool intersectPlane(Ray *ray, Intersection *intersection, Object *obj) {
  bool res = false;
  vec3 O = ray->orig;
  vec3 d = ray->dir;
  vec3 n = obj->geom.plane.normal;
  float dist = obj->geom.plane.dist;
  //! \todo : compute intersection of the ray and the plane object
  //produit scalaire
  float d_n = dot(d, n);
  //on verifie si dir . n != 0
  if (d_n != 0) {
    float t = -((dot(O, n)+ dist)/d_n);
    if (ray->tmin < t && ray->tmax > t) { // on est dans l'intervalle
      intersection->position = rayAt(*ray, t); //ray->orig+t*obj->geom.plane.dist;
      intersection->mat = &(obj->mat);
      intersection->normal = n;
      ray->tmax = t; //mise à jour du tmax pour evité l'effet de bord
      res = true;
    }
  }
  return res;
}

//resoution d'equation du second degré
bool solver(float a, float b, float c, float*t) {
  bool res = false;
  //delta
  float delta = b*b - 4*a*c;
  if (delta > 0) {
    float s1 = (-b-sqrt(delta))/(2*a);
    if (s1 > 0) {
      *t = s1;
    }else {
      *t = (-b+sqrt(delta))/2*a;
    }
    res = true;
  }
  return res;
}

bool intersectSphere(Ray *ray, Intersection *intersection, Object *obj) {
  bool res = false;
  float t = 0.0;
  vec3 O = ray->orig;
  vec3 d = ray->dir;
  vec3 C = obj->geom.sphere.center;
  float R = obj->geom.sphere.radius;
    //! \todo : compute intersection of the ray and the sphere object
  float a = 1.0;
  float b = 2*(dot(d, (O - C)));
  float c = (dot((O- C), (O - C)) - (R*R));
  res = solver(a, b, c, &t);
  if (res) { //on cherche si on a une intersection
    if(ray->tmin < t && ray->tmax > t) {
      point3 p = rayAt(*ray, t);
      intersection->position = p; //ray->orig+t*ray->dir;
      intersection->mat = &(obj->mat);
      intersection->normal = normalize(p-C);
      ray->tmax = t; //mise à jour du tmax pour evité l'effet de bord
    }else {
      res = false;
    }
  }
  return res;
}

bool intersectScene(const Scene *scene, Ray *ray, Intersection *intersection) {
  bool hasIntersection = false;
  size_t objectCount = scene->objects.size();
  for ( size_t i=0; i<objectCount; i++) {
    Etype type = scene->objects[i]->geom.type;
    switch (type) {
      case PLANE:
        if (intersectPlane(ray, intersection, scene->objects[i])) {
          hasIntersection = true;
        }
        break;
      case SPHERE:
        if (intersectSphere(ray, intersection, scene->objects[i])) {
          hasIntersection = true;
        }
        break;
      case TRIANGLE:
        if (intersectTriangle(ray, intersection, scene->objects[i])) {
          hasIntersection = true;
        }
        break;
      case CYLINDRE:
        if (intersectCylindre(ray, intersection, scene->objects[i])) {
          hasIntersection = true;
        }
        break;
      case CERCLE:
        if (intersectCercle(ray, intersection, scene->objects[i])) {
          hasIntersection = true;
        }
        break;
    }
  }
  //!\todo loop on each object of the scene to compute intersection
  return hasIntersection;
}

/* --------------------------------------------------------------------------- */
/*
 *	The following functions are coded from Cook-Torrance bsdf model description and are suitable only
 *  for rough dielectrics material (RDM. Code has been validated with Mitsuba renderer)
 */

/** Normal Distribution Function : Beckmann
 * NdotH : Norm . Half
 */
float RDM_Beckmann(float NdotH, float alpha) {
  float cos2 = NdotH*NdotH;
  float alpha2 = alpha*alpha;
  float tan2 = (1-cos2) / cos2;
  float cos4 = cos2*cos2;
  float D = exp(-tan2/alpha2)/(pi*alpha2*cos4);
  //! \todo compute Beckmann normal distribution
  //return 0.5f;
  return D;
}

// Fresnel term computation. Implantation of the exact computation. we can use the Schlick approximation
// LdotH : Light . Half
float RDM_Fresnel(float LdotH, float extIOR, float intIOR) {
  float cos2Oi = LdotH*LdotH;
  float sin2Ot = ((extIOR/intIOR)*(extIOR/intIOR))*(1-cos2Oi);
  if (sin2Ot > 1) {
    return 1;
  }
  //assert(sin2Ot<=1 && sin2Ot>=-1);
  float cosOt = sqrt(1-sin2Ot);
  float Rs = ((extIOR*LdotH - intIOR*cosOt)*(extIOR*LdotH - intIOR*cosOt)) /
             ((extIOR*LdotH + intIOR*cosOt)*(extIOR*LdotH + intIOR*cosOt));
  float Rp = ((extIOR*cosOt - intIOR*LdotH)*(extIOR*cosOt - intIOR*LdotH)) /
             ((extIOR*cosOt + intIOR*LdotH)*(extIOR*cosOt + intIOR*LdotH));
  float F = (Rs+Rp)/2;
  //! \todo compute Fresnel term
  // return 0.5f;
  return F;

}


// Shadowing and masking function. Linked with the NDF. Here, Smith function, suitable for Beckmann NDF
float RDM_chiplus(float c) {
  return (c > 0.f) ? 1.f : 0.f;
}

// DdotH : Dir . Half
// HdotN : Half . Norm
float RDM_G1(float DdotH, float DdotN, float alpha) {
  float cosOx = DdotN; // car c'est l'opposer de l'autre vecteur un doute a verifier
  float cos2Ox = cosOx*cosOx;
  float tanOx = sqrt(1-cos2Ox)/cos2Ox;
  float b = 1/(alpha*tanOx);
  float k = DdotH/DdotN;
  float G1 = 0.0;
  if (k > 0 && b < 1.6) {
    G1 = (3.535 * b + 2.181 * b * b) / (1 + 2.276 * b + 2.577 * b * b);
  }else {
     G1 = RDM_chiplus(k);
    // if (k > 0) {
    //   G1 = 1.0;
    // }
  }
  //!\todo compute G1 term of the Smith fonction
  //return 0.5f;
  return G1;
}

// LdotH : Light . Half
// LdotN : Light . Norm
// VdotH : View . Half
// VdotN : View . Norm
float RDM_Smith(float LdotH, float LdotN, float VdotH, float VdotN, float alpha) {
  float G = 0.5f;
  //!\todo the Smith fonction
  float G1l = RDM_G1(LdotH, LdotN, alpha);
  float G1v = RDM_G1(VdotH, VdotN, alpha);
  G = G1l*G1v;
  return G;


}

// Specular term of the Cook-torrance bsdf
// LdotH : Light . Half
// NdotH : Norm . Half
// VdotH : View . Half
// LdotN : Light . Norm
// VdotN : View . Norm
color3 RDM_bsdf_s(float LdotH, float NdotH, float VdotH, float LdotN, float VdotN, Material *m) {
  float alpha = m->roughness;
  color3 Ks = m->specularColor;
  float extIOR = 1; // on part du principe que c'est l'aire
  float intIOR = m->IOR;
  float D = RDM_Beckmann(NdotH, alpha);
  float F = RDM_Fresnel(LdotH, extIOR, intIOR);
  float G = RDM_Smith(LdotH, LdotN, VdotH, VdotN, alpha);
  color3 BSDF_s = Ks * ((D * F * G) / (4 * LdotN * VdotN));
  return BSDF_s;
  //!\todo specular term of the bsdf, using D = RDB_Beckmann, F = RDM_Fresnel, G = RDM_Smith
  // return color3(.5f);
}
// diffuse term of the cook torrance bsdf
color3 RDM_bsdf_d(Material *m) {
  return m->diffuseColor/pi;
  //!\todo compute diffuse component of the bsdf
  //return color3(.5f);
}

// The full evaluation of bsdf(wi, wo) * cos (thetai)
// LdotH : Light . Half
// NdotH : Norm . Half
// VdotH : View . Half
// LdotN : Light . Norm
// VdtoN : View . Norm
// compute bsdf * cos(Oi)
color3 RDM_bsdf(float LdotH, float NdotH, float VdotH, float LdotN, float VdotN, Material *m) {
  color3 BSDF = (RDM_bsdf_d(m) + RDM_bsdf_s(LdotH, NdotH, VdotH, LdotN, VdotN, m));
  //! \todo compute bsdf diffuse and specular term
  //return color3(0.f);
  return BSDF;

}




/* --------------------------------------------------------------------------- */

color3 shade(vec3 n, vec3 v, vec3 l, color3 lc, Material *mat ){
  color3 ret = color3(0.f);
  //color3 ret = color3(0,0,0);
  float LdotN = dot(l, n);
  if (LdotN > 0) {
    vec3 h = normalize((v+l)/length(v+l)); // je ne suis   bsolument pas sur : half-vector
    float LdotH = dot(l, h);
    float NdotH = dot(n, h);
    float VdotH = dot(v, h);
    float VdotN = dot(v, n);
    ret = lc*RDM_bsdf(LdotH, NdotH, VdotH, LdotN, VdotN, mat)*LdotN;
    // ret = (mat->diffuseColor*LdotN*lc) / pi;
  }

  //! \todo compute bsdf, return the shaded color taking into account the
  //! lightcolor
  return ret;

}

//! if tree is not null, use intersectKdTree to compute the intersection instead of intersect scene
color3 trace_ray(Scene * scene, Ray *ray, KdTree *tree) {
  color3 ret = color3(0,0,0);
  Intersection intersection;
  if (intersectScene(scene, ray, &intersection)) {
    // vec3 c = 0.5f*(intersection.normal)+0.5f;
    // ret = color3(c.x, c.y, c.z); //première coloration
    // return ret;
    vec3 l;
    vec3 rr;
    vec3 cr;
    Light * lgt;
    vec3 v = -(ray->dir);
    color3 lc;
    vec3 n = intersection.normal; //pourquoi essaie ici
    point3 p = intersection.position;
    Ray newRay;
    Ray refRay;
    size_t lightCount = scene->lights.size();
    for ( size_t i=0; i<lightCount; i++) {
      lgt = scene->lights[i];
      l = normalize(lgt->position-p);
      newRay.orig =  p+acne_eps*(l);
      newRay.dir = l;
      newRay.tmin = 0; // le min est à 0
      newRay.tmax = (lgt->position.x - p.x)/(newRay.dir).x; //nouveau max pour le rayon
      lc = lgt->color;
      Intersection interOmbre;
      if (!intersectScene(scene, &newRay, &interOmbre)) {
        ret += shade(n, v, l, lc, intersection.mat);
      }
    }

    if (ray->depth < NbRebond) {
      rr = normalize(reflect(ray->dir, intersection.normal)); // peut etre inverser les parametre pour voir
      //calcul du rayon secondaire
      vec3 newOrig = p+acne_eps*(rr);
      rayInit(&refRay, newOrig, rr, 0, 10000, ray->depth+1);
      //refRay.depth +=1; //on incremente le nombre de rebon
      cr = trace_ray(scene, &refRay, tree);

      //on verifie que le cr ne depasse pas 1, si c'est le cas on le ramene à 1 car trop de reflection sion
      if (cr.x > 1) cr.x = 1;
      if (cr.y > 1) cr.y = 1;
      if (cr.z > 1) cr.z = 1;

      vec3 h = normalize(v + rr);
      l = rr;
      float LdotH = dot(l, h);
      return ret+RDM_Fresnel(LdotH, 1.f, intersection.mat->IOR)*cr*intersection.mat->specularColor;
    }else {
      return color3(0,0,0);
    }
  }else {
    return scene->skyColor;
  }
}

//pour tester le kdTree si besoin il n'y a pas d'erreur de compilation
// color3 trace_ray(Scene * scene, Ray *ray, KdTree *tree) {
//   color3 ret = color3(0,0,0);
//   Intersection intersection;
//   if (intersectKdTree(scene, tree, ray, &intersection) || intersectSceneAfterKdTree(scene, tree, ray, &intersection)) {
//     // vec3 c = 0.5f*(intersection.normal)+0.5f;
//     // ret = color3(c.x, c.y, c.z); //première coloration
//     // return ret;
//     vec3 l;
//     vec3 rr;
//     vec3 cr;
//     Light * lgt;
//     vec3 v = -(ray->dir);
//     color3 lc;
//     vec3 n = intersection.normal; //pourquoi essaie ici
//     point3 p = intersection.position;
//     Ray newRay;
//     Ray refRay;
//     size_t lightCount = scene->lights.size();
//     for ( size_t i=0; i<lightCount; i++) {
//       lgt = scene->lights[i];
//       l = normalize(lgt->position-p);
//       newRay.orig =  p+acne_eps*(l);
//       newRay.dir = l;
//       newRay.tmin = 0; // le min est à 0
//       newRay.tmax = (lgt->position.x - p.x)/(newRay.dir).x; //nouveau max pour le rayon
//       lc = lgt->color;
//       Intersection interOmbre;
//       if (!intersectKdTree(scene, tree, ray, &interOmbre)) {
//         ret += shade(n, v, l, lc, intersection.mat);
//       }
//     }
//
//     if (ray->depth < NbRebond) {
//       rr = normalize(reflect(ray->dir, intersection.normal)); // peut etre inverser les parametre pour voir
//       //calcul du rayon secondaire
//       vec3 newOrig = p+acne_eps*(rr);
//       rayInit(&refRay, newOrig, rr, 0, 10000, ray->depth+1);
//       //refRay.depth +=1; //on incremente le nombre de rebon
//       cr = trace_ray(scene, &refRay, tree);
//
//       //on verifie que le cr ne depasse pas 1, si c'est le cas on le ramene à 1 car trop de reflection sion
//       if (cr.x > 1) {
//         cr.x = 1;
//       }
//       if (cr.y > 1) {
//         cr.y = 1;
//       }
//       if (cr.z > 1) {
//         cr.z = 1;
//       }
//
//       vec3 h = normalize(v + rr);
//       l = rr;
//       float LdotH = dot(l, h);
//       return ret+RDM_Fresnel(LdotH, 1.f, intersection.mat->IOR)*cr*intersection.mat->specularColor;
//     }else {
//       return color3(0,0,0);
//     }
//   }else {
//     return scene->skyColor;
//   }
// }

void renderImage(Image *img, Scene *scene) {

  //! This function is already operational, you might modify it for antialiasing and kdtree initializaion
  float aspect = 1.f/scene->cam.aspect;

  KdTree *tree =  NULL;
  //decommenter si on veut utiliser le kdtree
  // tree = initKdTree(scene);
  // printf("feuille = %d \n", tree->root->leaf);
  //! \todo initialize KdTree

  float delta_y = 1.f / (img->height * 0.5f); //! one pixel size
  vec3 dy = delta_y * aspect * scene->cam.ydir; //! one pixel step
  vec3 ray_delta_y = (0.5f - img->height * 0.5f) / (img->height * 0.5f) * aspect * scene->cam.ydir;

  float delta_x = 1.f / (img->width * 0.5f);
  vec3 dx = delta_x * scene->cam.xdir;
  vec3 ray_delta_x = (0.5f - img->width * 0.5f) / (img->width * 0.5f) *scene->cam.xdir;


  for(size_t j=0; j<img->height; j++) {
    if(j!=0) printf("\033[A\r");
    float progress = (float)j/img->height*100.f;
    printf("progress\t[");
    int cpt = 0;
    for(cpt = 0; cpt<progress; cpt+=5) printf(".");
    for(       ; cpt<100; cpt+=5) printf(" ");
    printf("]\n");
#pragma omp parallel for
    for(size_t i=0; i<img->width; i++) {
      color3 *ptr = getPixelPtr(img, i,j);
      vec3 centre = scene->cam.center + ray_delta_x + ray_delta_y;
      *ptr += anti_aliasing(scene, tree, NbRayon, dx, dy, centre, i, j);
    }
  }
}

//renvoie rand entre deux born
float frand_a_b(float a, float b){
    a = a*100.f;
    b = b*100.f;
    return ((rand()/(float)RAND_MAX ) * (b-a) + a)/100.f;
}

color3 anti_aliasing(Scene *scene, KdTree *tree, int n, vec3 dx, vec3 dy, vec3 centre, int i, int j) {
  Ray rx;
  vec3 ray_dir;
  color3 ptr;

  for (float k = -0.5f+(1.f/(2*n)); k < 0.5f; k += (1.f/n)) {
    for (float p = -0.5f+(1.f/(2*n)); p < 0.5f; p += (1.f/n)){
      /*decommenter les ligne qui suivent pour l'antialiasing avec points aleatoire sur un interval*/
      // float k1 = frand_a_b(k, k+(1.f/n));
      // float p1 = frand_a_b(p, p+(1.f/n));
      // ray_dir = centre + float(i+p1)*dx + float(j+k1)*dy;

      //commenter cette ligne si vous utiliser l'anti_aliasing normal
      ray_dir = centre + float(i+p)*dx + float(j+k)*dy;
      rayInit(&rx, scene->cam.position, normalize(ray_dir));
      ptr += trace_ray(scene, &rx, tree);
    }
  }
  ptr = (1.f/(n*n))*(ptr);
  return ptr;
}
