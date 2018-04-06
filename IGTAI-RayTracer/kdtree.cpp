#include "kdtree.h"
#include "defines.h"
#include "scene.h"
#include "scene_types.h"
#include <stdio.h>
#include "raytracer.h"

#include <vector>
#include <stack>

#define DLIMIT 3
#define OLIMIT 10

typedef struct s_kdtreeNode KdTreeNode;

struct s_kdtreeNode {
  bool leaf; //! is this node a leaf ?
  int axis;//! axis index of the split, if not leaf
  float split;//!position of the split
  int depth; //!depth in the tree
  std::vector<int> objects;//! index of objects, if leaf
  KdTreeNode* left;//!ptr to left child
  KdTreeNode* right;//! ptr to right child
  vec3 min;//! min pos of node bounding box
  vec3 max;//! max pos of node bounding box
};

KdTreeNode * initNode(bool l, int a, int d) {
    KdTreeNode *ret = new KdTreeNode();
    ret->leaf = l;
    ret->axis = a;
    ret->depth = d;
    ret->left = NULL;
    ret->right = NULL;
    return ret;
}
//==================================================================

typedef struct s_stackNode {
    float tmin;
    float tmax;
    KdTreeNode *node;
} StackNode;

  // StackNode * stackNode() {
  //   StackNode s_node = new stackNode();
  //   s_node->tymin = 0;
  //   s_node->tymax = 0;
  //   s_node->node = NULL;
  //   retun s_node;
  // }
  //
  // StackNode * push_node(StackNode * s_node, KdTreeNode node, float tymin, float tymax) {
  //
  // }

//==================================================================
struct s_kdtree {
    int depthLimit;
    size_t objLimit;
    KdTreeNode *root;

    std::vector<int> outOfTree;
    std::vector<int> inTree;
};
//==================================================================
void subdivide(Scene *scene, KdTree *tree, KdTreeNode *node);
bool intersectAabb(Ray *theRay,  vec3 min, vec3 max);


void calculatenewMinMax(Scene *scene, std::vector<int> indObjPlan, vec3 min, vec3 max) {
  vec3 a, b, c;// car triangle
  Objects obj = scene->objects;
  size_t objectCount = indObjPlan.size();
  for ( size_t i=0; i<objectCount; i++) {
    a = obj[indObjPlan[i]]->geom.triangle.pointA; //recuperation des point
    b = obj[indObjPlan[i]]->geom.triangle.pointB;
    c = obj[indObjPlan[i]]->geom.triangle.pointC;
    if (c.x < min.x) min.x = c.x;
    if (c.x > max.x) max.x = c.x;

    if (c.y < min.y) min.y = c.y;
    if (c.y > max.y) max.y = c.y;

    if (c.z < min.z) min.z = c.z;
    if (c.z > max.z) max.z = c.z;

    if (a.x < min.x) min.x = a.x;
    if (a.x > max.x) max.x = a.x;

    if (a.y < min.y) min.y = a.y;
    if (a.y > max.y) max.y = a.y;

    if (a.z < min.z) min.z = a.z;
    if (a.z > max.z) max.z = a.z;

    if (b.x < min.x) min.x = b.x;
    if (b.x > max.x) max.x = b.x;

    if (b.y < min.y) min.y = b.y;
    if (b.y > max.y) max.y = b.y;

    if (b.z < min.z) min.z = b.z;
    if (b.z > max.z) max.z = b.z;

  }
}


int splite_func(int a) {return (a+1)%3;} // retourn l'axe sur la quel on coup 0->x 1->y 2->z

//pour l'instant j'ai choisir de le faire qu'avec des object triangle
KdTree*  initKdTree(Scene *scene) {
    KdTree* kdtree = new KdTree(); // declaration du kdtree
    // KdTreeNode * root = kdtree->root;
    kdtree->root = initNode(false, 0, 0); // pour l'instant on es à zero deplacement
    // printf("feuille = %d \n", kdtree->root->leaf);
    kdtree->depthLimit = DLIMIT;
    kdtree->objLimit = OLIMIT;

    Objects obj = scene->objects;
    Etype type;
    //initilisation de min et max
    kdtree->root->min = vec3(1000000.f, 1000000.f, 1000000.f); //on initilialiste à l'objet 0 comme on est sur d'etre
    kdtree->root->max = vec3(-1000000.f, -1000000.f, -1000000.f); //dans les objet (triangle)
    vec3 a, b, c;// car triangle
    size_t objectCount = obj.size(); //on parcour l'essemble des objects
    for (size_t i=0; i<objectCount; i++) {
      type = obj[i]->geom.type;
      if (type == TRIANGLE ) { //on traite les coordonee du triangle
        a = obj[i]->geom.triangle.pointA; //recuperation des point
        b = obj[i]->geom.triangle.pointB;
        c = obj[i]->geom.triangle.pointC;
        //recherche du minimum
        if (c.x < kdtree->root->min.x) kdtree->root->min.x = c.x;
        if (c.x > kdtree->root->max.x) kdtree->root->max.x = c.x;

        if (c.y < kdtree->root->min.y) kdtree->root->min.y = c.y;
        if (c.y > kdtree->root->max.y) kdtree->root->max.y = c.y;

        if (c.z < kdtree->root->min.z) kdtree->root->min.z = c.z;
        if (c.z > kdtree->root->max.z) kdtree->root->max.z = c.z;

        if (a.x < kdtree->root->min.x) kdtree->root->min.x = a.x;
        if (a.x > kdtree->root->max.x) kdtree->root->max.x = a.x;

        if (a.y < kdtree->root->min.y) kdtree->root->min.y = a.y;
        if (a.y > kdtree->root->max.y) kdtree->root->max.y = a.y;

        if (a.z < kdtree->root->min.z) kdtree->root->min.z = a.z;
        if (a.z > kdtree->root->max.z) kdtree->root->max.z = a.z;

        if (b.x < kdtree->root->min.x) kdtree->root->min.x = b.x;
        if (b.x > kdtree->root->max.x) kdtree->root->max.x = b.x;

        if (b.y < kdtree->root->min.y) kdtree->root->min.y = b.y;
        if (b.y > kdtree->root->max.y) kdtree->root->max.y = b.y;

        if (b.z < kdtree->root->min.z) kdtree->root->min.z = b.z;
        if (b.z > kdtree->root->max.z) kdtree->root->max.z = b.z;
        //on ajoute l'object dans trouvé treein
        kdtree->inTree.push_back(i);
        kdtree->root->objects.push_back(i); // on saisit les element dans le noeud de depart
      }else {
        kdtree->outOfTree.push_back(i);
      }
    }
    subdivide(scene, kdtree, kdtree->root); // appelle recurcif sur la division de scene
    //!\todo compute scene bbox, store object in outOfTree or inTree depending on type
    // printf("je quite bien la fonction subdivide\n");
    return kdtree;

}


//from http://blog.nuclex-games.com/tutorials/collision-detection/static-sphere-vs-aabb/
bool intersectSphereAabb(vec3 sphereCenter, float sphereRadius, vec3 aabbMin, vec3 aabbMax) {
    vec3 closestPointInAabb = min(max(sphereCenter, aabbMin), aabbMax);
    vec3 seg = closestPointInAabb -  sphereCenter;
    float distanceSquared = dot(seg, seg);
    // The AABB and the sphere overlap if the closest point within the rectangle is
    // within the sphere's radius
    return distanceSquared < (sphereRadius * sphereRadius);
}


void subdivide(Scene *scene, KdTree *tree, KdTreeNode *node) {
  if (tree->objLimit >= node->objects.size() || node->depth == tree->depthLimit) {//cas d'arret on test si on peut diviser ou pas
    printf("%d\n", node->objects.size());
    node->leaf = true; //c'est donc une feuille on retourne le noeud
    return;
  }else {// c'est donc pas une feuille on continue donc
    //j'initialise 2 fils
    //trouver le bon plan de decoupage
    node->left = initNode(false, splite_func(node->axis), node->depth+1); //on incremente la proondeur et on change d'axe de decoupage
    node->right = initNode(false, splite_func(node->axis), node->depth+1);
    //recherche du point de decoupage
    float mil = (node->min[node->axis]+node->max[node->axis])/2 ;//vec3((node->min.x+node->max.x)/2, (node->min.y+node->max.y)/2, (node->min.z+node->max.z)/2);
    node->left->min = node->min;
    node->left->max = node->max;
    node->left->max[node->axis] = mil;
    node->right->max = node->max;
    node->right->min = node->min;
    node->right->min[node->axis] = mil;
    node->split = mil; //mi à jour du splite
    //separation des object
    point3 a, b, c;
    Objects obj = scene->objects;
    // Etype type;
    size_t objectCount = node->objects.size(); //on parcour l'essemble des objects
    for (size_t i=0; i<objectCount; i++) { //on remplie la feuille
        a = obj[node->objects[i]]->geom.triangle.pointA; //recuperation des point
        b = obj[node->objects[i]]->geom.triangle.pointB;
        c = obj[node->objects[i]]->geom.triangle.pointC;

        if (a[node->axis] <= mil || (b[node->axis] <= mil) || (c[node->axis] <= mil)) //{
          node->left->objects.push_back(i);

        if (a[node->axis] >= mil || (b[node->axis] >= mil) || c[node->axis] >= mil) //{
          node->right->objects.push_back(i);
    }
    subdivide(scene, tree, node->left);
    subdivide(scene, tree, node->right);
  }
    //!\todo generate children, compute split position, move objets to children and subdivide if needed.
}

// bool traverse1(Scene * scene, KdTree * tree, std::stack<StackNode> *stack, StackNode currentNode, Ray * ray, Intersection *intersection) {
//
//     if (intersectAabb(ray, tree->root->min, tree->root->max)) {
//       currentNode.tmin = ray->tmin;
//       currentNode.tmax = ray->tmax;
//     }
//     // if (/* condition */) {
//     //   /* code */
//     // }
//     //! \todo traverse kdtree to find intersection
//     return false;
// }

bool traverse(Scene * scene, KdTreeNode * node, Ray * ray, Intersection * intersection) {
  if (node->leaf) {
      size_t objectCount = node->objects.size();
      bool res = false;
      for (size_t i = 0; i < objectCount; i++) {
        if (intersectTriangle(ray, intersection, scene->objects[node->objects[i]])) {
          res = true;
        }
      }
      return res;
  }else { //c'est pas une feuille
    //calcul du splite
    float splite = (node->split - ray->orig[node->axis])/ray->dir[node->axis]; //on calcule le splite
    //recherche du fils le plus proche
    bool res = false;
    if(ray->tmax < splite) { // c'est le fis gauche qui est intersecté
      res = traverse(scene, node->left, ray, intersection);
    }else if(ray->tmin > splite) { //c'est le fils droit qui est intersecté
      res = traverse(scene, node->right, ray, intersection);
    }else if ( ray->tmin <= splite && splite <= ray->tmax){// o est dans le cas ou il travere les deux
        // if (!(res =  traverse(scene, node->left, ray, intersection))) //ce ci est un essaie
          res = traverse(scene, node->left, ray, intersection);
          res = traverse(scene, node->right, ray, intersection);
    }
    return res;
  }
}

// from http://www.scratchapixel.com/lessons/3d-basic-lessons/lesson-7-intersecting-simple-shapes/ray-box-intersection/
bool intersectAabb(Ray *theRay,  vec3 min, vec3 max) {
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    vec3 bounds[2] = {min, max};
    tmin = (bounds[theRay->sign[0]].x - theRay->orig.x) * theRay->invdir.x;
    tmax = (bounds[1-theRay->sign[0]].x - theRay->orig.x) * theRay->invdir.x;
    tymin = (bounds[theRay->sign[1]].y - theRay->orig.y) * theRay->invdir.y;
    tymax = (bounds[1-theRay->sign[1]].y - theRay->orig.y) * theRay->invdir.y;
    if ((tmin > tymax) || (tymin > tmax)) return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;
    tzmin = (bounds[theRay->sign[2]].z - theRay->orig.z) * theRay->invdir.z;
    tzmax = (bounds[1-theRay->sign[2]].z - theRay->orig.z) * theRay->invdir.z;
    if ((tmin > tzmax) || (tzmin > tmax)) return false;
    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;
    if (tmin > theRay->tmin) theRay->tmin = tmin;
    if (tmax < theRay->tmax) theRay->tmax = tmax;
    // printf("je suis sortie vois tu\n");
    return true;
}


bool intersectKdTree(Scene *scene, KdTree *tree, Ray *ray, Intersection *intersection) {
    bool hasIntersection = false;

    if (intersectAabb(ray, tree->root->min, tree->root->max)) {
      // printf("hasIntersection with box\n");
      if (traverse(scene, tree->root, ray, intersection)) {
        // printf("hasIntersection tri\n");
        hasIntersection = true;
      }else
        if (intersectSceneAfterKdTree(scene, tree, ray, intersection)) hasIntersection = true;
    }else
      if (intersectSceneAfterKdTree(scene, tree, ray, intersection)) hasIntersection = true;
    //!\todo call vanilla intersection on non kdtree object, then traverse the tree to compute other intersections
    return hasIntersection;
}

bool intersectSceneAfterKdTree(Scene *scene, KdTree *tree, Ray *ray, Intersection *intersection) {
  bool hasIntersection = false;
  size_t objectCount = tree->outOfTree.size();
  for ( size_t i=0; i<objectCount; i++) {
    Etype type = scene->objects[tree->outOfTree[i]]->geom.type;
    switch (type) {
      case PLANE:
        if (intersectPlane(ray, intersection, scene->objects[tree->outOfTree[i]])) {
          hasIntersection = true;
        }
        break;
      case SPHERE:
        if (intersectSphere(ray, intersection, scene->objects[tree->outOfTree[i]])) {
          hasIntersection = true;
        }
        break;
      case TRIANGLE:
        if (intersectTriangle(ray, intersection, scene->objects[tree->outOfTree[i]])) {
          hasIntersection = true;
        }
        break;
      case CYLINDRE:
        if (intersectCylindre(ray, intersection, scene->objects[tree->outOfTree[i]])) {
          hasIntersection = true;
        }
        break;
      case CERCLE:
        if (intersectCercle(ray, intersection, scene->objects[tree->outOfTree[i]])) {
          hasIntersection = true;
        }
        break;
    }
  }
  return hasIntersection;
}
