#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include "defines.h"
#include "ray.h"
#include "scene.h"
#include "raytracer.h"
#include "image.h"
#include "scene_types.h"
#include <time.h>

#define WIDTH 800
#define HEIGHT 600

// #define NbV 773//426//star 3276//43 //loup 547
// #define NbF 1508//star 6516//22//loup 956
/* nickel :
 mat.diffuseColor = color3(0.014, 0.012, 0.012);
 mat.specularColor  = color3(1.0, 0.882, 0.786);
 mat.IOR = 2.4449;
 mat.roughness = 0.0681;
 */

/* specular black phenolic :
	mat.diffuseColor = color3(0.002, 0.002, 0.003);
	mat.specularColor  = color3(1.0, 0.824, 0.945);
	mat.IOR = 1.072;
	mat.roughness = 0.0588;
	*/

/* specular blue phenolic :
 mat.diffuseColor = color3(0.005,0.013,0.032);
 mat.specularColor  = color3(1.0, 0.748, 0.718);
 mat.IOR = 1.1051;
 mat.roughness = 0.0568;
 */

/* specular green phenolic :
 mat.diffuseColor = color3(0.006, 0.026, 0.022);
 mat.specularColor  = color3(1.0, 0.739, 0.721);
 mat.IOR = 1.1051;
 mat.roughness = 0.0567;
 */

/* specular white phenolic :
	mat.diffuseColor = color3(0.286, 0.235, 0.128);
	mat.specularColor  = color3(1.0, 0.766, 0.762);
	mat.IOR = 1.1022;
	mat.roughness = 0.0579;
	*/

/* marron plastic :
 mat.diffuseColor = color3(0.202, 0.035, 0.033);
 mat.specularColor  = color3(1.0,0.857,0.866);
 mat.IOR = 1.0893;
 mat.roughness = 0.0604;
 */

/* purple paint :
 mat.diffuseColor = color3(0.301, 0.034, 0.039);
 mat.specularColor  = color3(1.0, 0.992, 0.98);
 mat.IOR = 1.1382;
 mat.roughness = 0.0886;
 */
/* red specular plastic :
 mat.diffuseColor = color3(0.26, 0.036, 0.014);
 mat.specularColor  = color3(1.0, 0.852, 1.172);
 mat.IOR = 1.0771;
 mat.roughness = 0.0589;
 */

/* green acrylic :
 mat.diffuseColor = color3(0.016, 0.073, 0.04);
 mat.specularColor  = color3(1.0, 1.056, 1.146);
 mat.IOR = 1.1481;
 mat.roughness = 0.0625;
 */

/* blue acrylic :
 mat.diffuseColor = color3(0.012,0.036,0.106);
 mat.specularColor  = color3(1.0, 0.965, 1.07);
 mat.IOR = 1.1153;
 mat.roughness = 0.068;
 */

int split(char * coord) {
  char res[10];
  for (size_t i = 0; i < strlen(coord); i++) {
    if (coord[i] != '/') {
      res[i] = coord[i];
    }else {
      res[i] = '\0';
      i = strlen(coord);
    }
  }
  return atoi(res);
}

Object ** loupEnMaillage(const char * filename, Material mat, float miseAlEchelle, int NbF, int NbV){
    //ouverture du fichier de lecture
    FILE * fic = NULL;
    if ((fic = fopen(filename, "r")) == NULL) {
      fprintf(stderr, "%s n'existe pas\n", filename);
      exit(EXIT_FAILURE);
    }

    //initilisation des object
    Object ** ensObj = (Object **)malloc(sizeof(Object*)*NbF); //[956];
    // float tabCoord[NbV][3];
    point3 tabCoord[NbV];
    char marque[40];
    char coord1[20], coord2[20], coord3[20];

    //lecture des coordonnee
    for (int i = 1; i < NbV; i++) {
      // if(fscanf(fic, "%s %s %s", coord1, coord2, coord3) == 0) exit(EXIT_FAILURE);
      if(fscanf(fic, "%f %f %f", &tabCoord[i].x, &tabCoord[i].y, &tabCoord[i].z) == 0) exit(EXIT_FAILURE);
      tabCoord[i].x = tabCoord[i].x/miseAlEchelle;
      tabCoord[i].y = tabCoord[i].y/miseAlEchelle;
      tabCoord[i].z = tabCoord[i].z/miseAlEchelle;
    }
    if(fscanf(fic, "%s", marque) == 0) exit(EXIT_FAILURE);
    point3 a, b, c;
    if (strcmp(marque, "#finCoordonnee") == 0) {
      // #pragma omp parallel for
      for (int i = 0; i <NbF; i++) {
        if(fscanf(fic,"%s %s %s", coord1, coord2, coord3) == 0) exit(EXIT_FAILURE);
        a = tabCoord[split(coord1)];
        b = tabCoord[split(coord2)];
        c = tabCoord[split(coord3)];
        ensObj[i] = initTriangle(b, a, c, mat);
      }
    }
    // printf("fin de lecture du fichier\n");
    fclose(fic);
    return ensObj;
}

void insererUnObjetEnMaillage(Scene *scene, const char * filename, int nombreDeFace, int nombreDePoint, float miseAlEchelle, Material mat) {
  Object **obj = loupEnMaillage(filename, mat, miseAlEchelle, nombreDeFace, nombreDePoint);
  for (int i = 0; i < nombreDeFace; i++) {
    addObject(scene, obj[i]);
  }
}
//quadrilataire
void addQuadrilatair(Scene * scene, point3 A, point3 B, point3 C, point3 position, Material mat1, Material mat2) {
  addObject(scene, initTriangle(A, B, C, mat1));
  point3 mil = point3((A.x+B.x)/2, (A.y+B.y)/2, (A.z+B.z)/2); //x1+x2/2 ...
  point3 D = point3(2*mil.x-C.x, 2*mil.y-C.y, 2*mil.z-C.z); //2*m1-x3 ...
  addObject(scene, initTriangle(A, B, D, mat2));
}



//ajoute un cylindre en fonction de son orientation
void addCylinder(Scene * scene, float r, float h, point3 centre, int orientation, Material mat) {
  addObject(scene, initCylindre(r, h, centre, orientation, mat));
  switch (orientation) {
    case 0:
      addObject(scene, initCercle(r, centre, vec3(0, 1, 0), mat)); //base inferieur
      addObject(scene, initCercle(r, point3(centre.x, centre.y+h, centre.z), vec3(0, 1, 0), mat)); // base superieur
      break;
    case 1:
      addObject(scene, initCercle(r, centre, vec3(1, 0, 0), mat)); //base inferieur
      addObject(scene, initCercle(r, point3(centre.x+h, centre.y, centre.z), vec3(1, 0, 0), mat)); // base superieur
      break;
    case 2:
      addObject(scene, initCercle(r, centre, vec3(0, 0, 1), mat)); //base inferieur
      addObject(scene, initCercle(r, point3(centre.x, centre.y, centre.z+h), vec3(0, 0, 1), mat)); // base superieur
      break;
  }

}

void addPyramide(Scene* scene, point3 a, point3 b, point3 c, point3 f, point3 i, Material mat) {
  point3 d = a;  point3 j = a;
  point3 e = c;  point3 k = i;
  point3 g = a;  point3 l = b;
  point3 m = b;  point3 n = c;
  point3 o = i;  point3 p = c;
  point3 q = f;  point3 h = f;
  point3 r = f;
  addObject(scene, initTriangle(a, b, c, mat));
  addObject(scene, initTriangle(d, e, f, mat));
  addObject(scene, initTriangle(g, h, i, mat));
  addObject(scene, initTriangle(j, k, l, mat));
  addObject(scene, initTriangle(m, n, o, mat));
  addObject(scene, initTriangle(p, q, r, mat));
}

Scene * initScene0() {
    Scene *scene = initScene();
    setCamera(scene, point3(3,1,0), vec3(0,0.3,0), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
    setSkyColor(scene, color3(0.1f, 0.3f, 0.5f));
    Material mat;
    mat.IOR = 1.3;
    mat.roughness = 0.1;
    mat.specularColor = color3(0.5f);

    mat.diffuseColor = color3(.5f);
    addObject( scene, initSphere(point3(0,0, 0),0.25, mat));


    mat.diffuseColor = color3(0.5f, 0.f, 0.f);
    addObject( scene, initSphere(point3(1,0, 0),.25, mat));


    mat.diffuseColor = color3(0.f, 0.5f, 0.5f);
    addObject( scene, initSphere(point3(0,1, 0),.25, mat));

    mat.diffuseColor = color3(0.f, 0.f, 0.5f);
    addObject( scene, initSphere(point3(0,0,1),.25, mat));


    mat.diffuseColor  = color3(0.6f);
    addObject(scene, initPlane(vec3(0,1,0), 0, mat));

    addLight(scene, initLight(point3(10, 10,10), color3(1,1,1)));
    addLight(scene, initLight(point3(4, 10,-2), color3(1,1,1)));

    return scene;
}

Scene * initScene1() {

    Scene *scene = initScene();
    setCamera(scene, point3(3,0,0), vec3(0,0.3,0), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
	setSkyColor(scene, color3(0.2, 0.2, 0.7));

    Material mat;
    mat.IOR = 1.12;
    mat.roughness = 0.2;
    mat.specularColor = color3(0.4f);
    mat.diffuseColor  = color3(0.6f);

    for(int i=0; i<10; ++i) {
        mat.diffuseColor = color3(0.301, 0.034, 0.039);
        mat.specularColor  = color3(1.0, 0.992, 0.98);
        mat.IOR = 1.1382;
        mat.roughness = 0.0886;
        mat.roughness = ((float)10-i)/(10*9.f);
        addObject( scene, initSphere(point3(0,0, -1.5+i/9.f*3.f),.15, mat));
    }
    for(int i=0; i<10; ++i) {
        mat.diffuseColor = color3(0.012,0.036,0.106);
        mat.specularColor  = color3(1.0, 0.965, 1.07);
        mat.IOR = 1.1153;
        mat.roughness = 0.068;
        mat.roughness = ((float)i+1)/10.f;
        addObject( scene, initSphere(point3(0,1, -1.5+i/9.f*3.f),.15, mat));
    }
    mat.diffuseColor = color3(0.014, 0.012, 0.012);
    mat.specularColor  = color3(1.0, 0.882, 0.786);
    mat.IOR = 2.4449;
    mat.roughness = 0.0681;
    addObject( scene, initSphere(point3(-3.f, 1.f, 0.f),2., mat));

    mat.diffuseColor = color3(0.016, 0.073, 0.04);
    mat.specularColor  = color3(1.0, 1.056, 1.146);
    mat.IOR = 1.1481;
    mat.roughness = 0.0625;
    addObject(scene, initPlane(vec3(0,1,0), +1, mat));

    addLight(scene, initLight(point3(10, 10,10), color3(10,10,10)));
    addLight(scene, initLight(point3(4, 10,-2), color3(5,3,10)));
    return scene;
}


Scene *initScene2() {
    Scene *scene = initScene();
    setCamera(scene, point3(0.5,3,1), vec3(0,0,0.6), vec3(0,0,1), 60, (float)WIDTH/(float)HEIGHT);
    setSkyColor(scene, color3(0.2, 0.2, 0.7));
    Material mat;
    mat.diffuseColor = color3(0.014, 0.012, 0.012);
    mat.specularColor  = color3(1.0, 0.882, 0.786);
    mat.IOR = 2.4449;
    mat.roughness = 0.0681;


    mat.diffuseColor = color3(0.05,0.05,0.05);
    mat.specularColor  = color3(0.95);
    mat.IOR = 1.1022;
    mat.roughness = 0.0579;

    addObject(scene, initPlane(vec3(0,0,1), 0, mat));


    mat.diffuseColor = color3(0.005,0.013,0.032);
    mat.specularColor  = color3(1.0, 0.748, 0.718);
     for(int i=0; i<4; ++i) {
        mat.IOR = 1.1051+(-0.1+float(i)/3.f*0.4);
        for(int j=0; j<10; ++j) {
            mat.roughness = 0.0568+(-0.1+float(j)/9.f*0.3);
            addObject( scene, initSphere(point3(-1.5+float(j)/9.f*3.f,0, 0.4+float(i)*0.4f),.15, mat));
        }
    }
    addLight(scene, initLight(point3(-20, 5,10), color3(30,30,30)));
    addLight(scene, initLight(point3(10, 10,10), color3(30,30,30)));
    addLight(scene, initLight(point3(50, -100,10), color3(1,0.7,2)));
    return scene;



}

Scene* initScene3(){
    Scene *scene = initScene();
    setCamera(scene, point3(4.5,.8,4.5), vec3(0,0.3,0), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
    setSkyColor(scene, color3(0.2, 0.2, 0.7));
    Material mat;
    mat.diffuseColor = color3(0.301, 0.034, 0.039);
    mat.specularColor  = color3(1.0, 0.992, 0.98);
    mat.IOR = 1.1382;
    mat.roughness = 0.0886;

    addLight(scene, initLight(point3(0, 1.7, 1), .5f*color3(3,3,3)));
    addLight(scene, initLight(point3(3, 2, 3),   .5f*color3(4, 4, 4)));
    addLight(scene, initLight(point3(4,3,-1),    .5f*color3(5, 5, 5)));




    mat.diffuseColor = color3(0.014, 0.012, 0.012);
    mat.specularColor  = color3(0.7, 0.882, 0.786);
    mat.IOR = 6;
    mat.roughness = 0.0181;
    addObject( scene, initSphere(point3(0,0.1,0),.3, mat));

    mat.diffuseColor = color3(0.26, 0.036, 0.014);
    mat.specularColor  = color3(1.0, 0.852, 1.172);
    mat.IOR = 1.3771;
    mat.roughness = 0.01589;
    addObject( scene, initSphere(point3(1,-.05,0),.15, mat));

    mat.diffuseColor = color3(0.014, 0.012, 0.012);
    mat.specularColor  = color3(0.7, 0.882, 0.786);
    mat.IOR = 3;
    mat.roughness = 0.00181;
    addObject( scene, initSphere(point3(3,0.05,2),.25, mat));

    mat.diffuseColor = color3(0.46, 0.136, 0.114);
    mat.specularColor  = color3(0.8, 0.852, 0.8172);
    mat.IOR = 1.5771;
    mat.roughness = 0.01589;
    addObject( scene, initSphere(point3(1.3,0.,2.6),0.215, mat));

    mat.diffuseColor = color3(0.06, 0.26, 0.22);
    mat.specularColor  = color3(0.70, 0.739, 0.721);
    mat.IOR = 1.3051;
    mat.roughness = 0.567;
    addObject( scene, initSphere(point3(1.9,0.05,2.2),.25, mat));

    mat.diffuseColor = color3(0.012,0.036,0.406);
    mat.specularColor  = color3(1.0, 0.965, 1.07);
    mat.IOR = 1.1153;
    mat.roughness = 0.068;
    mat.roughness = 0.18;
    addObject( scene, initSphere(point3(0,0,1),.20, mat));

    mat.diffuseColor = color3(.2,0.4,.3);
    mat.specularColor = color3(.2,0.2,.2);
    mat.IOR = 1.382;
    mat.roughness = 0.05886;
    addObject(scene, initPlane(vec3(0,1,0),0.2, mat));

    mat.diffuseColor = color3(.5,0.09,.07);
    mat.specularColor = color3(.2,.2,.1);
    mat.IOR = 1.8382;
    mat.roughness = 0.886;
    addObject(scene, initPlane(vec3(1, 0.0, -1.0), 2, mat));


    mat.diffuseColor = color3(0.1,0.3,.05);
    mat.specularColor = color3(.5,.5,.5);
    mat.IOR = 1.9382;
    mat.roughness = 0.0886;
    addObject(scene, initPlane(vec3(0.3,-0.2, 1), 4, mat));
    return scene;
}

Scene* initScene5(){
  Scene *scene = initScene();
  setCamera(scene, point3(3,1,0), vec3(-1, 0.6, 0), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
  setSkyColor(scene, color3(0.1f, 0.3f, 0.5f));
  Material mat;
  mat.IOR = 1.3;
  mat.roughness = 0.1;
  mat.specularColor = color3(0.5f);
  mat.diffuseColor = color3(.5f);
  mat.diffuseColor = color3(0.5f, 0.f, 0.f);
  addCylinder(scene, 0.1, 0.5, point3(0.5, 0.5, 0.2), 1, mat);
  addCylinder(scene, 0.1, 0.5, point3(0.5, 0.5, 0.2), 2, mat);
  addCylinder(scene, 0.1, 0.5, point3(0.5, 0.5, 0.2), 0, mat);
  addCylinder(scene, 0.1, 0.5, point3(0.5, 0.5, -0.2-0.1), 2, mat);
  addCylinder(scene, 0.1, 0.5, point3(0.5, 0, 0.2), 0, mat);
  addObject( scene, initSphere(point3(0.5, 1, 0.2),.2, mat));
  addObject( scene, initSphere(point3(0.5, 0.5, 0.2),.2, mat));
  addObject( scene, initSphere(point3(0.5, 0.5, -0.3),.1, mat));
  addObject( scene, initSphere(point3(0.5, 0.5, 0.7),.1, mat));
  addObject( scene, initSphere(point3(1, 0.5, 0.2),.05, mat));
  addObject(scene, initCercle(.4, point3(0.5, 0.5, 0.2), point3(0, 1, 0), mat));
  addCylinder(scene, 0.1, 0.5, point3(-3, 0.5, -3), 2, mat);
  addCylinder(scene, 0.1, 0.5, point3(-3, 0.6, -3), 2, mat);
  addCylinder(scene, 0.1, 0.5, point3(-3, 0.7, -3), 2, mat);
  addCylinder(scene, 0.1, 0.5, point3(-3, 0.8, -3), 2, mat);
  mat.diffuseColor  = color3(0.8f);
  addObject(scene, initCercle(.4, point3(-2, 0.9, -2), point3(0, 0.3, 1), mat));

  mat.diffuseColor = color3(0.1,0.3,.05);
  mat.specularColor = color3(.5,.5,.5);
  mat.IOR = 1.9382;
  mat.roughness = 0.0886;
  addObject(scene, initPlane(vec3(0,1,0), 0, mat));

  mat.diffuseColor = color3(.2,0.4,.3);
  mat.specularColor = color3(.2,0.2,.2);
  mat.IOR = 2.382;
  mat.roughness = 0.5886;
  addObject(scene, initPlane(vec3(0,0,1), 3, mat));

  addLight(scene, initLight(point3(4, 10,-2), color3(1,1,1)));
  addLight(scene, initLight(point3(10, 10,10), color3(1,1,1)));
  addLight(scene, initLight(point3(20, 40,5), color3(1,1,1)));
  return scene;
}

Scene* initScene6() {
  Scene *scene = initScene();
  // setCamera(scene, point3(2,1, 7), vec3(1, 0.6, .6), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
  setCamera(scene, point3(3,1,0), vec3(0,0.3,0), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
  setSkyColor(scene, color3(0.1f, 0.3f, 0.5f));
  Material mat;
  mat.diffuseColor = color3(0.301, 0.034, 0.039);
  mat.specularColor  = color3(1.0, 0.992, 0.98);
  mat.IOR = 1.1382;
  mat.roughness = 0.0886;;

  insererUnObjetEnMaillage(scene, "loup.txt" , 957, 547, 200, mat);

  mat.diffuseColor  = color3(0.6f);
  addObject(scene, initPlane(vec3(0,1,0), 0, mat));
  addLight(scene, initLight(point3(40, 100,-20), color3(1,1,1)));
  addLight(scene, initLight(point3(100, 100,100), color3(1,1,1)));
  addLight(scene, initLight(point3(200, 400,50), color3(1,1,1)));
  return scene;
}

Scene * initScene4() {

    Scene *scene = initScene();
    // setCamera(scene, point3(3,0,0), vec3(0,0.3,0), vec3(0,1,0), 70, (float)WIDTH/(float)HEIGHT);
    setCamera(scene, point3(2,1, 7), vec3(1, 0.5, 0), vec3(0,1,0), 60, (float)WIDTH/(float)HEIGHT);
	   setSkyColor(scene, color3(0.2, 0.2, 0.7));

    Material mat;
    mat.IOR = 1.12;
    mat.roughness = 0.2;
    mat.specularColor = color3(0.4f);
    mat.diffuseColor  = color3(0.6f);

    insererUnObjetEnMaillage(scene, "../Data/storne.txt" , 6516, 3276, 200, mat);


    mat.diffuseColor = color3(0.014, 0.012, 0.012);
    mat.specularColor  = color3(1.0, 0.882, 0.786);
    mat.IOR = 2.4449;
    mat.roughness = 0.0681;
    addObject( scene, initSphere(point3(-3.f, 1.f, 0.f),2., mat));

    mat.diffuseColor = color3(0.016, 0.073, 0.04);
    mat.specularColor  = color3(1.0, 1.056, 1.146);
    mat.IOR = 1.1481;
    mat.roughness = 0.0625;
    addObject(scene, initPlane(vec3(0,1,0), +1, mat));

    addLight(scene, initLight(point3(10, 10,10), color3(10,10,10)));
    addLight(scene, initLight(point3(4, 10,-2), color3(5,3,10)));
    return scene;
}


int main(int argc, char *argv[]) {
    srand(time(NULL));
    printf("Welcom to the L3 IGTAI RayTracer project\n");

    char basename[256];

    if(argc<2 || argc >3) {
        printf("usage : %s filename i\n", argv[0]);
        printf("        filename : where to save the result, whithout extention\n");
        printf("        i : scenen number, optional\n");
        exit(0);
    }

    strncpy(basename, argv[1], 256);

    int scene_id = 0;
    if(argc == 3) {
        scene_id = atoi(argv[2]);
    }

    Image *img = initImage(WIDTH,HEIGHT);
    Scene * scene = NULL;
    switch (scene_id) {
    case  0 :
        scene = initScene0();
        break;
    case  1 :
        scene = initScene1();
        break;
    case  2 :
        scene = initScene2();
        break;
    case  3 :
        scene = initScene3();
        break;
    case  4 :
        scene = initScene4();
        break;
    case  5 :
        scene = initScene5();
        break;
    default :
        scene = initScene0();
        break;
    }

    printf("render scene %d\n", scene_id);

    renderImage(img, scene);
    freeScene(scene);
    scene = NULL;

    printf("save image to %s\n", basename);
    saveImage(img, basename);
    freeImage(img);
    img = NULL;
    printf("done. Goodbye\n");

    return 0;
}
