bool intersectTriangle2(Ray *ray, Intersection *intersection, Object *obj) {
  vec3 v0 = obj->geom.triangle.pointA;
  vec3 v1 = obj->geom.triangle.pointB;
  vec3 v2 = obj->geom.triangle.pointC;

  vec3 n = cross<float>((v1-v0), (v2-v0));
  n = normalize<float>(n);

  float cos_theta = dot<float>(n,ray->dir);
  if (cos_theta == 0) return false;

  float D = dot<float>(-n, v0);

  float t = -(dot<float>(n, ray->orig)+D)/cos_theta;
  if (t < 0 || t < ray->tmin || t > ray->tmax) return false;

  vec3 position = rayAt(*ray, t);

  vec3 edge0 = v1-v0;
  vec3 edge1 = v2-v1;
  vec3 edge2 = v0-v2;

  vec3 pv0 = position - v0;
  vec3 pv1 = position - v1;
  vec3 pv2 = position - v2;

  if (dot<float>(n, cross<float>(edge0, pv0)) > 0 &&
      dot<float>(n, cross<float>(edge1, pv1)) > 0 &&
      dot<float>(n, cross<float>(edge2, pv2)) > 0) {

    intersection->normal = n;
    intersection->position = position;
    intersection->mat = &(obj->mat);
    ray->tmax = t;
    return true;
  }

  return false;
}

bool intersectTriangle1(Ray *ray, Intersection *intersection, Object *obj) {

  vec3 v0 = obj->geom.triangle.pointA;
  vec3 v1 = obj->geom.triangle.pointB;
  vec3 v2 = obj->geom.triangle.pointC;

  float a, u, v, f;
  vec3 e1, e2, h, s, q;
  e1 = v1-v0;
  e2 = v2-v0;

  h = cross(ray->dir, e2);
  a = dot(e1, h);
  if (a > -EPSILON && a < EPSILON) return false;

  f = 1/a;

  s = ray->orig - v0;
  u = f * dot(s, h);
  if (u < 0.0 || u > 1.0) return false;

  q = cross(s, e1);
  v = f*dot(ray->dir, q);
  if (v < 0.0 || u+v > 1) return false;

  float t = f*dot(e2, q);
  if (t > ray->tmin && t < ray->tmax) {
    intersection->normal = normalize(cross(e2, e1));
    intersection->position = rayAt(*ray, t);
    intersection->mat = &(obj->mat);
    ray->tmax = t;
    return true;
  }
  return false;
}



//CYLINDRE
vec3 point = rayAt(*ray, t);
vec3 normal = vec3(2*point.x, 0.0f, 2*point.z);

//on est dans le cas du cylindre fini y doit etre compris entre 0 et hauteur cylindre
if (point.y < 0 || point.y > h) return false;

//dans le cas ou le rayon est parallele au cylindre on plusieur cas
if (dir.y != 0.0f) {
    float t3 = (0-O.y)/dir.y;
    float t4 = (h-O.y)/dir.y;
    float t2;
    //on choisit le plus petit >= 0
    t2 = std::min(t3,t4);
    if (t2 < 0) {
        t2 = std::max(t3,t4);
    }

    if (t2 >= 0) {
        vec3 point1 = O + dir*t2;

        if (point1.x*point1.x + point1.z*point1.z <= R*R+0.9f) {
            //on choisis le plus petit des intersection entre l'ancien t et le nouveau t2
            t = std::min(t,t2);
            if (t == t3) {
                normal = vec3(0.0f,-1.0f,0.0f);
            }else if (t == t4) {
                normal = vec3(0.0f,1.0f,0.0f);
            }
            point = point1;
        }
    }
}

intersection->normal = normal;
intersection->position = point;
intersection->mat = &(obj->mat);
ray->tmax = t;
return true;







bool intersectCylindre1(Ray *ray, Intersection *intersection, Object *obj) {

    vec3 p = ray->orig; //point d'origine du rayon
    vec3 d = ray->dir; //direction du rayon
    float r = obj->geom.cylindre.radius; //rayon
    float h = obj->geom.cylindre.h; // hauteur du cylindre si = 0 cylindre infini

    float t = 0.0f; // inconnue cherchee pour resoudre la position
    //declaration des parametre de l'equation à resoudre
    float a = d.y*d.y+d.z*d.z; //d1²+d2²
    float b = 2*(d.y*p.y+d.z*p.z); //d1*p1+d2*p2
    float c = p.y*p.y+p.z*p.z-r*r; //p1²+p2²-rayon
    vec3 position;
    vec3 n;
    // printf("je rentre dedans\n");

    //resolution de l'equation pour trouver t on prends le plus petit
    if(!solver(a, b, c, &t)) { // delat < 0
        if (p.y*p.y+p.z*p.z <= r*r) return false; // on est dans l'alignement du cylindre

        //rayon rencontre la face superieur du cylindre
        if (h > 0 && (p.x > h || (p.x < h && p.x > 0))) {
            //l'intersection se calcule comme pour le plane
            if (d.x < 0) return false; //on fait ça que si d0 est positif

            t = (h-p.x)/d.x;
            // position = rayAt(*ray, t); //calcul de la position
            n = vec3(-1.0f, 0.0f, 0.0f);
        //on racontre la face inferieur
        }else if (h > 0 && ((p.x > 0 || p.x < 0)&& p.x < h)) {
            if (d.x < 0) return false;
            t = -p.x/d.x;;
            n = vec3(1.0f, 0.0f, 0.0f);
        }
    }else {
        position = rayAt(*ray, t);
        n = vec3(0.0f, 2*position.x, 2*position.z);;
    }

    if (t > ray->tmin && t < ray->tmax) {
        //calcule de ma normal
        intersection->normal = n;
        intersection->position = position;
        intersection->mat = &(obj->mat);
        ray->tmax = t;
        return true;
    }

    return false;
}















//dernier mis a jour
bool intersectCylindre1(Ray *ray, Intersection *intersection, Object *obj) {

    vec3 p = ray->orig; //point d'origine du rayon
    vec3 d = ray->dir; //direction du rayon
    float r = obj->geom.cylindre.radius; //rayon
    float h = obj->geom.cylindre.h; // hauteur du cylindre si = 0 cylindre infini

    float t = 0.0f; // inconnue cherchee pour resoudre la position
    //declaration des parametre de l'equation à resoudre
    float a = d.x*d.x+d.z*d.z; //d1²+d2²
    float b = 2*(d.x*p.x+d.z*p.z); //d1*p1+d2*p2
    float c = p.x*p.x+p.z*p.z-r*r; //p1²+p2²-rayon
    vec3 position;
    vec3 n;
    // printf("je rentre dedans\n");

    //resolution de l'equation pour trouver t on prends le plus petit
    if(!solver(a, b, c, &t)) { // delat < 0
        if (p.x*p.x+p.z*p.z <= r*r) { // on est dans l'alignement du cylindre
        //rayon rencontre la face superieur du cylindre
            if (h > 0 && (p.y > h || (p.y < h && p.y > 0))) {
                //l'intersection se calcule comme pour le plane
                if (d.y < 0) return false; //on fait ça que si d0 est positif

                t = (h-p.y)/d.y;
                position = rayAt(*ray, t); //calcul de la position
                position.y = h;
                n = vec3(1.0f, 0.0f, 0.0f);
            //on racontre la face inferieur
            }else if (h > 0 && ((p.y > 0 || p.y < 0)&& p.y < h)) {
                if (d.y < 0) return false;
                t = -p.y/d.y;;
                position = rayAt(*ray, t); //calcul de la position
                position.y = 0;
                n = vec3(-1.0f, 0.0f, 0.0f);
            }
            // position = rayAt(*ray, t);
        }
    }else {
        position = rayAt(*ray, t);
        n = vec3(2*position.x, 0.f, 2*position.z);;
    }

    if (t > ray->tmin && t < ray->tmax) {
        //calcule de ma normal
        intersection->normal = normalize(n);
        intersection->position = position;
        intersection->mat = &(obj->mat);
        ray->tmax = t;
        return true;
    }

    return false;
}


bool intersectCylindre(Ray *ray, Intersection *intersection, Object *obj) {

    vec3 O = ray->orig;
    // vec3 O = vec3(-1, 0, 0);
    vec3 dir = ray->dir;
    float R = obj->geom.cylindre.radius;
    float h = obj->geom.cylindre.h;

    //resolution d'equation
    float a = dir.x*dir.x + dir.z*dir.z;
    float b = 2*(O.x*dir.x + O.z*dir.z);
    float c = O.x*O.x + O.z*O.z - R*R;
    //deltat calcul
    float t = 0.0;
    if(!solver(a, b, c, &t)) return false;
    // float delt = b*b-4*a*c;
    // if (det < 0) return false;
    //calcule de la nouvelle position et de la normal
    vec3 point = rayAt(*ray, t);
    vec3 normal = vec3(1*point.x, 0.0f, 1*point.z);

    //on est dans le cas du cylindre fini y doit etre compris entre 0 et hauteur cylindre
    if (point.y < 0 || point.y > h) return false;

    //dans le cas ou le rayon est parallele au cylindre on plusieur cas
    if (dir.y != 0.0f) {
        float t3 = (0-O.y)/dir.y;
        float t4 = (h-O.y)/dir.y;
        float t2;
        //on choisit le plus petit >= 0
        t2 = std::min(t3,t4);
        if (t2 < 0) {
            t2 = std::max(t3,t4);
        }

        if (t2 >= 0) {
            vec3 point1 = O + dir*t2;

            if (point1.x*point1.x + point1.z*point1.z <= R*R+0.9) {
                //on choisis le plus petit des intersection entre l'ancien t et le nouveau t2
                t = std::min(t,t2);
                if (t == t3) {
                    normal = vec3(0.0f,-1.0f,0.0f);
                }else if (t == t4) {
                    normal = vec3(0.0f,1.0f,0.0f);
                }
                point = point1;
            }
        }
    }
    if (t > ray->tmin && t < ray->tmax) {
        intersection->normal = normalize(normal);
        intersection->position = point;
        intersection->mat = &(obj->mat);
        ray->tmax = t;
        return true;
    }
    return false;
}



bool intersectCone(Ray *ray, Intersection *intersection, Object *obj) {
    vec3 O = ray->orig;
    vec3 dir = ray->dir;
    float cosa;
    // float cos2 = cost*cost;

    point3 P; //point d'intersection
    vec3 H = obj->geom.cone.vecteur;
    point3 C = obj->geom.cone.centre; //C
    float r = obj->geom.cone.radius; //r
    // float h = obj->geom.cone.h;
    vec3 w = O-H;
    vec3 h = C - H;
    vec3 ht = 1/length(h)*h;
    float m = (r*r)/(length(h)*length(h));

    float a = dot(dir, dir)*dot(dir, dir) - m*(dot(dir, ht)*dot(dir, ht)) - (dot(dir, ht)*dot(dir, ht));
    float b = 2*(dot(dir, w) - m*(dot(dir, ht)*dot(w, ht)) - (dot(dir, ht)*dot(w, ht)));
    float c = (dot(w, w)* dot(w, w)) - m*(dot(w, ht)*dot(w, ht)) - (dot(w, ht)*dot(w, ht));
    float t;
    float d = b*b-4*a*c;

    if (d < 0) return false; //delatat degatuve

    if (d == 0) { //bcp de cas
      cosa = length(h)/sqrt(length(h)*length(h) + r*r);
      vec3 vt = 1/length(dir)*dir;
      if(cosa == dot(vt, ht)) return false;
      t = -b/2*a;
    }else {//delata > 0
      t = (-b-sqrt(d))/(2*a);
      if (t < 0) {
        t = (-b+sqrt(d))/2*a;
      }
    }
    P = rayAt(*ray, t);

    // if (0 < dot((P-C), h) || dot((P-C), h) <= length(h)) return false;
    // if ( )
    float hauteur = length(C-H);
    if (ray->tmin < t && ray->tmax > t) { // on est dans l'intervalle
      float R = P.y;
      if (R <= C.y || R >= C.y + hauteur) return false;
      R =sqrt((P.x-C.x)*(P.x-C.x)+(P.z-C.z)*(P.z-C.z));
      intersection->position = P; //ray->orig+t*obj->geom.plane.dist;
      intersection->mat = &(obj->mat);
      intersection->normal = point3(P.x-C.x,R*(r/hauteur),P.z-C.z);
      ray->tmax = t; //mise à jour du tmax pour evité l'effet de bord
      return true;
    }
    return false;
}
