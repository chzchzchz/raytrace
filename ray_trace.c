#include <math.h>
#include <sys/time.h>
#include <string.h>
#include <assert.h>
#include "ray_trace.h"

extern void readSceneFile(char *filename);
static void ray_trace_scene(void);

SceneInfo scene;		/* information read from the file */
static int scan_line = 0;	/* used for debugging */

#define SUB_PIXELS	3	/* helps scene 2 polygon intersection */
#define MAX_DEPTH	15	/* number of levels of recursion in tracer */
#define WIN_SIZE	400	/* pixels per line */

/* illumination model */
#define REFRACT_COEF	0.2
#define REFLECT_COEF	0.3
#define DIFFUSE_COEF	0.9
#define AMBIENT_LIGHT	0.04
#define SPECULAR_COEF	0.9
#define SHINE_FACTOR	50

static int win_size_x = WIN_SIZE;	/* number of pixels per line in view window */
static int win_size_y = WIN_SIZE;	/* number of pixels per line in view window */
static Color	**bitmap;	/* holds the color information for raytrace */


static void norm_vector(Vector* v);
static double v_dist(const Vertex* v, const Vertex *v2);
static double dot(const Vector* v1, const Vector* v2);
static Ray reflect(const Ray* r, const Vector* n, const Vertex* int_pt);
static Vector cross(const Vector* v1, const Vector* v2);
static double dist_from_ray(const Ray* r, const Vertex* v);
static Color apply_shadows(Ray* r, Solid* s, Vertex* v, Vector* n);
static Color apply_shadow(Ray *r, Solid* s, Vertex* v, Vector* n, Light* l);
static void pixel_to_world(int x, int y, double *w_x, double *w_y);
static int poly_intersect(void* obj, Ray* r, Vertex* v);
static int poly_intersect_2d(Poly* p, double u, double v);
static Vector poly_normal(void* obj, const Vertex* v);
static void poly_init(void* obj);
static int sphere_intersect(void* obj, Ray* r, Vertex* v);
static Vector sphere_normal(void* obj, const Vertex* v);
static void sphere_init(void* obj);
static Color ray_trace(Ray* r, int depth);
static Color render_pixel(int i, int j);
static void init_solids(void);

/* solid functions on a polygon */
SolidFuncs poly_funcs;
/* solid functions on a sphere */
SolidFuncs sphere_funcs;

/* 
 * msvc doesn't like initializing structures by field name!
 * lovely fixup follows
 */
static void init_solid_funcs(void)
{
	poly_funcs.intersect = poly_intersect;
	poly_funcs.normal = poly_normal;
	poly_funcs.init = poly_init;

	sphere_funcs.intersect = sphere_intersect;
	sphere_funcs.normal = sphere_normal;
	sphere_funcs.init = sphere_init;
}

/*
*  rayTrace
*  This is the function that draws the raytrace for the scene specified in 
*  the scene file 
*/
void draw_raytrace(void)
{
	int i, j;

	glClear(GL_COLOR_BUFFER_BIT);
	
	// Now draw all the pixels
	glBegin(GL_POINTS);
	for(i = 0; i < win_size_x; i++){
		for(j = 0; j < win_size_y; j++){
			glColor3f(	bitmap[i][j].r, 
					bitmap[i][j].g, bitmap[i][j].b);
			glVertex2i(i, j);
		}
	}
	glEnd();
	glFlush();

	glutSwapBuffers();
}

static void update(int n)
{
	ray_trace_scene();
	glutPostRedisplay();
	/* we could do some animation here if we wanted */
	// scene.view_z += 1;
	glutTimerFunc(1, update, 1);
}

/* 
 * creates a reflected ray given an incoming ray, a surface normal, and
 * a point of incidence
 */
static Ray reflect(const Ray* r, const Vector* normal, const Vertex* int_pt)
{
	Ray	reflected;

	reflected.origin = *int_pt;
	reflected.dir.x = -2.0*dot(normal, &r->dir)*normal->x + r->dir.x;
	reflected.dir.y = -2.0*dot(normal, &r->dir)*normal->y + r->dir.y;
	reflected.dir.z = -2.0*dot(normal, &r->dir)*normal->z + r->dir.z;

	return reflected;
}

#define sign(x)	((x < 0) ? -1 : 1)

/* determines if (u,v) falls in the u,v polygon */
/* ripped from slides */
/* returns 0 on failure to intersect */
static int poly_intersect_2d(Poly* p, double u, double v)
{
	int	a;
	int	sgn;
	int	num_crossings;

	sgn = sign(p->poly_2d[1][0] - v);
	num_crossings = 0;
	for(a = 0; a < p->no_vertices; a++){
		int	b;
		int	next_sgn;
		double	u_a, u_b, v_b, v_a;
		
		b = (a + 1) % p->no_vertices;

		u_a = p->poly_2d[0][a] - u;
		u_b = p->poly_2d[0][b] - u;
		v_a = p->poly_2d[1][a] - v;
		v_b = p->poly_2d[1][b] - v;

		next_sgn = sign(p->poly_2d[1][b] - v);
		if(next_sgn == sgn)
			continue;

		if(u_a > 0 && u_b > 0){
			num_crossings++;		
		}else if(u_a > 0 || u_b > 0){
			double	i;
			i = u_a - v_a*(u_b - u_a) / (v_b - v_a);
			if(i > 0)
				num_crossings++;
		}
		sgn = next_sgn;
	}

	/* odd number of crosses, we're in the polygon */
	if((num_crossings % 2) == 1){
		return 2;
	}

	/* even number-- not in the polygon */
	return 0;
}


/* calculate the euclidean distance between two vertices */
static double v_dist(const Vertex* v, const Vertex *v2)
{
	return sqrt(	(v->x - v2->x)*(v->x - v2->x) + 
			(v->y - v2->y)*(v->y - v2->y) + 
			(v->z - v2->z)*(v->z - v2->z));
}

/* determine whether a ray intersects a polygon */
/* assign closest point of intersection to 'pt' */
static int poly_intersect(void* obj, Ray* r, Vertex* pt)
{
	Poly	*p;
	double t;
	double	origin_dot, dir_dot;
	double	u, v;

	p = obj;
	origin_dot = dot(&p->normal, &r->origin);
	dir_dot = dot(&p->normal, &r->dir);

	/* avoid singularity for planes right on the ray */
	if(fabs(dir_dot) < 1.0e-20)
		return 0;

	/* do we need to reverse the normal? */
	if(dir_dot > 0){
		origin_dot = -origin_dot;
	}

	t = -(origin_dot + p->d) / dir_dot;
	if(t < 0)
		return 0;

	pt->x = r->origin.x + r->dir.x*t;
	pt->y = r->origin.y + r->dir.y*t;
	pt->z = r->origin.z + r->dir.z*t;
	/* ok, we now know that we intersect the plane */
	/* now verify that we are in the polygon */

	/* translate intersection point to u,v */
	if(p->gone_coord == 0){
		u = pt->y;
		v = pt->z;
	}else if(p->gone_coord == 1){
		u = pt->x;
		v = pt->z;
	}else{
		u = pt->x;
		v = pt->y;
	}

	/* translated system => (u', v') = (u_n - u, v_n - v) */
	/* putting (u,v) at the origin */
	return poly_intersect_2d(p, u, v);
}

/* get the normal for a polygon at point 'v' */
/* assumes 'v' is on the polygon */
static Vector poly_normal(void* obj, const Vertex* v)
{
	Poly	*p;

	p = obj;

	return p->normal;
}

/* initialize polygon values that we'll need for later */
static void poly_init(void* obj)
{
	Poly	*p;
	Vector	n;
	double	dotted;
	Vector	v1, v2;
	int	i;

	p = obj;

	/* calculate the normal */
	/* all vertices should be on the same plane, 
	 * so the first three vertices should suffice */
	v1.x = p->vertices[2].x - p->vertices[0].x;
	v1.y = p->vertices[2].y - p->vertices[0].y;
	v1.z = p->vertices[2].z - p->vertices[0].z;

	v2.x = p->vertices[1].x - p->vertices[0].x;
	v2.y = p->vertices[1].y - p->vertices[0].y;
	v2.z = p->vertices[1].z - p->vertices[0].z;

	n = cross(&v1, &v2);
	norm_vector(&n);

	memcpy(&p->normal, &n, sizeof(Vector));

	/* dot(n, v) + d = 0 */
	dotted = dot(&n, &p->vertices[2]);
	p->d = -dotted;

	/* construct intersection polygon by throwing away dominate coord */

	/* XXX this will probably be bad for the cache.. */
	p->poly_2d[0] = malloc(sizeof(float) * p->no_vertices);
	p->poly_2d[1] = malloc(sizeof(float) * p->no_vertices);

	if(fabs(n.x) >= fabs(n.y) && fabs(n.x) >= fabs(n.z))
		p->gone_coord = 0;
	else if(fabs(n.y) >= fabs(n.x) && fabs(n.y) >= fabs(n.z))
		p->gone_coord = 1;
	else
		p->gone_coord = 2;

	for(i = 0; i < p->no_vertices; i++){
		if(p->gone_coord == 0){
			/* x dominates */
			p->poly_2d[0][i] = p->vertices[i].y;
			p->poly_2d[1][i] = p->vertices[i].z;
		}else if(p->gone_coord == 1){
			/* y dominates */
			p->poly_2d[0][i] = p->vertices[i].x;
			p->poly_2d[1][i] = p->vertices[i].z;
		}else{
			/* z dominates */
			p->poly_2d[0][i] = p->vertices[i].x;
			p->poly_2d[1][i] = p->vertices[i].y;
		}
	}
}

/* dot product between two vectors */
static double dot(const Vector* v1, const Vector* v2)
{
	return v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;
}

/* cross product betewen two vectors */
static Vector cross(const Vector* v1, const Vector* v2)
{
	Vector	v;

	v.x = v1->y*v2->z - v1->z*v2->y;
	v.y = -(v1->x*v2->z - v1->z*v2->x);
	v.z = v1->x*v2->y - v1->y*v2->x;

	return v;
}

/* determine if a ray intersects a sphere */
/* assign the point of intersection to 'v' */
/* ripped from slides */
static int sphere_intersect(void* obj, Ray* r, Vertex* v)
{
	Sphere	*s;
	double	B, C;
	double	t_0, t_1, t_i;
	double	d, d_sqrt;
	int	n;

	s = obj;
	B = 2.0*(	r->dir.x*(r->origin.x - s->center.x) + 
			r->dir.y*(r->origin.y - s->center.y) + 
			r->dir.z*(r->origin.z - s->center.z));
	C = 	(r->origin.x - s->center.x)*(r->origin.x - s->center.x) +
		(r->origin.y - s->center.y)*(r->origin.y - s->center.y) +
		(r->origin.z - s->center.z)*(r->origin.z - s->center.z) -
		s->radius_sq;

	d = B*B - 4*C;
	/* invalid determinate for intersection? */
	if(d < 1e-20)
		return 0;

	d_sqrt = sqrt(d);
	t_0 = (-B - d_sqrt) / 2.0;
	t_1 = (-B + d_sqrt) / 2.0;

	/* make sure that we have positive t values so that we lie on ray */
	if(t_0 < 1.0e-200 && t_1 < 1.0e-200)
		return 0;

	/* find closest intersection point */
	if(t_0 < 1.0e-20){
		n = 1;
		t_i = t_1;
	}else{
		n = 2;
		t_i = t_0;
	}

	v->x = r->origin.x + r->dir.x*t_i;
	v->y = r->origin.y + r->dir.y*t_i;
	v->z = r->origin.z + r->dir.z*t_i;

	return n;
}

/* get the normal for a sphere given a point 'v' on the surface */
static Vector sphere_normal(void* obj, const Vertex* v)
{
	Sphere	*s;
	Vector	n;

	s = obj;

	n.x = (v->x - s->center.x) / s->radius;
	n.y = (v->y - s->center.y) / s->radius;
	n.z = (v->z - s->center.z) / s->radius;

	return n;
}

static void sphere_init(void* obj)
{
	Sphere*	s;
	s = obj;
	s->radius_sq = s->radius*s->radius;
}

/* viewing window is 2x2 on (-1,-1) -> (1,1) */
static void pixel_to_world(int x, int y, double *w_x, double *w_y)
{
	double	wsize_x,wsize_y;

	wsize_x = win_size_x;
	wsize_y = win_size_y;
	*w_x = -1.0 + 2.0*((double)x / wsize_x);
	*w_y = -1.0 + 2.0*((double)y / wsize_y);
}

/* normalize a vector (i.e. <v,v>/||v||) */
static void norm_vector(Vector* v)
{
	double	n;
	
	n = sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
	v->x /= n;
	v->y /= n;
	v->z /= n;
}

/* assume vertex falls on the ray */
/* return the value 't' that satisfies r(t) = v */
/* to make things easier for other apps, if t < 0, return a huge value */
static double dist_from_ray(const Ray* r, const Vertex* v)
{
	double	t;

	/* check to see if vertex falls behind ray */
	/* note 
		r->origin.x + r->dir.x*t = v->x;
		r->origin.y + r->dir.y*t = v->y;
		r->origin.z + r->dir.z*t = v->z;
	 */
	if(r->dir.z != 0)
		t = (v->z - r->origin.z) / r->dir.z;
	else if(r->dir.x != 0)
		t = (v->x - r->origin.x) / r->dir.x;
	else if(r->dir.y != 0)
		t = (v->y - r->origin.y) / r->dir.y;
	else
		t = -1;	/* you said this was a ray, but no direction vector? */

	return (t < 0) ? 1e307 : t;
}

/* trace a light's shadow ray on a solid */
#define MAX_PT_ERROR	1e-7
static Color apply_shadow(Ray* view, Solid* s, Vertex* v, Vector* n, Light* l)
{
	/* trace the light ray */
	Ray	r, reflected;
	Color	c;
	Vertex	v_closest;
	Solid	*s_closest;
	double	dist, d;
	double	shine;
	int	i;

	memset(&c, 0, sizeof(c));
	memcpy(&r.origin, &l->pos, sizeof(Vector));

	/* compute shadow ray between light and solid point */
	r.dir.x = (v->x - r.origin.x);
	r.dir.y = (v->y - r.origin.y);
	r.dir.z = (v->z - r.origin.z);
	norm_vector(&r.dir);

	/* find closest object to light on shadow ray */
	dist = 1e300;
	s_closest = NULL;
	memset(&v_closest, 0, sizeof(Vertex));
	for(i = 0; i < scene.solid_c; i++){
		Vertex	new_v;
		double	new_dist;
		int	int_tmp;
		
		int_tmp = scene.solids[i].f->intersect(scene.solids[i].obj, 
							&r, &new_v);
		if(!int_tmp)
			continue;
		
		new_dist = dist_from_ray(&r, &new_v);
		assert(new_dist < 1e100);
		if(new_dist < dist){
			s_closest = &scene.solids[i];
			dist = new_dist;
			v_closest = new_v;
		}
	}

	/* solid falls in a shadow, give nothing */
	if(s != s_closest){
		return c;
	}

	/* make sure we are near the point we wanted */
	dist = v_dist(&v_closest, v);
	if(dist > MAX_PT_ERROR)
		return c;


	memcpy(&c, &l->c, sizeof(Color));

	/* apply the Phong model */
	
	/* setup cos(theta) for Lambert's law */
	d = fabs(dot(&r.dir, n));
	
	/* setup specular */
	norm_vector(&view->dir);
	reflected = reflect(view, n, v);
	norm_vector(&reflected.dir);
	shine = pow(fabs(dot(&r.dir, &reflected.dir)), SHINE_FACTOR);

	c.r = s->c.r*c.r*DIFFUSE_COEF*d + c.r*SPECULAR_COEF*shine;
	c.g = s->c.g*c.g*DIFFUSE_COEF*d + c.g*SPECULAR_COEF*shine;
	c.b = s->c.b*c.b*DIFFUSE_COEF*d + c.b*SPECULAR_COEF*shine;
	return c;
}

/* trace all shadow rays for a point on a solid */
/* view = viewing ray, s = solid, v = point on solid, n = normal at point */
static Color apply_shadows(Ray* view, Solid* s, Vertex* v, Vector* n)
{
	int	i;
	Color	c;

	memset(&c, 0, sizeof(c));
	for(i = 0; i < scene.light_c; i++){
		Color	c_new;
		c_new = apply_shadow(view, s, v, n, &scene.lights[i]);
		c.r += c_new.r;
		c.g += c_new.g;
		c.b += c_new.b;
	}

	/* ambience */
	c.r += s->c.r * AMBIENT_LIGHT;
	c.g += s->c.g * AMBIENT_LIGHT;
	c.b += s->c.b * AMBIENT_LIGHT;

	/* clamp */
	if(c.r > 1.0)	c.r = 1.0;
	if(c.g > 1.0)	c.g = 1.0;
	if(c.b > 1.0)	c.b = 1.0;

	return c;
}


/* takes a ray, traces it on the scene to get a color */
static Color ray_trace(Ray* r, int depth)
{
	Solid	*s;
	Vertex	int_point;
	Color	color_reflect, color_local;
	Vector	normal;
	Ray	ray_reflect;
	//Ray	ray_refract;
	//Color	color_refract;
	double	dist = 1e300;
	int	i;

	/* clamp depth */
	if(depth > MAX_DEPTH){
		memset(&color_local, 0, sizeof(Color));
		return color_local;
	}

	/* find intersecting solid */
	s = NULL;
	for(i = 0; i < scene.solid_c; i++){
		Vertex	v;
		double	new_dist;
		
		if(!scene.solids[i].f->intersect(scene.solids[i].obj, r, &v))
			continue;
		
		new_dist = dist_from_ray(r, &v);
		assert(new_dist < 1e100);
		if(new_dist < dist){
			s = &scene.solids[i];
			dist = new_dist;
			int_point = v;
		}
	}

	/* no solid found => no light */
	if(s == NULL){
		memset(&color_local, 0, sizeof(Color));
		return color_local;
	}

	normal = s->f->normal(s->obj, &int_point);
	norm_vector(&normal);

	/* get lighting from point sources */
	color_local = apply_shadows(r, s, &int_point, &normal);

	ray_reflect = reflect(r, &normal, &int_point);

	/* peturb point by a little bit so we don't fall inside object */
	/* if there's no peturbation, the shadow rays tend to get unacceptable
	 * intersection point distances from the point we actually want */
	ray_reflect.origin.x += ray_reflect.dir.x*0.00001;
	ray_reflect.origin.y += ray_reflect.dir.y*0.00001;
	ray_reflect.origin.z += ray_reflect.dir.z*0.00001;

	/* get the light from reflection */
	color_reflect = ray_trace(&ray_reflect, depth + 1);

	//color_refract = ray_trace(ray_refract, depth + 1);

	color_local.r += 	REFLECT_COEF*color_reflect.r;
//				+ REFRACT_COEF*color_refract.r;
	color_local.g += 	REFLECT_COEF*color_reflect.g; 
//				+ REFRACT_COEF*color_refract.g;
	color_local.b += 	REFLECT_COEF*color_reflect.b;
//				+ REFRACT_COEF*color_refract.b;

	/* finally, clamp color values so we don't oversaturate by mistake */
	if(color_local.r > 1.0) color_local.r = 1.0;
	if(color_local.g > 1.0) color_local.g = 1.0;
	if(color_local.b > 1.0) color_local.b = 1.0;

	return color_local;
}

/* compute the color value for pixel (i,j) */
/* uses subpixel rendering */
static Color render_pixel(int i, int j)
{
	Ray	r;
	Color	retc;
	double	x, y, x2, y2;
	double	sub_x, sub_y;
	int	a, b;

	r.origin.x = 0;
	r.origin.y = 0;
	r.origin.z = scene.view_z;

	pixel_to_world(i, j, &x, &y);
	pixel_to_world(i + 1, j + 1, &x2, &y2);
	sub_x = (x2 - x) / ((float)SUB_PIXELS);
	sub_y = (y2 - y) / ((float)SUB_PIXELS);
	x -= sub_x;
	y -= sub_y;

	memset(&retc, 0, sizeof(Color));

	/* compuer and average all subpixels */
	for(a = 0; a < SUB_PIXELS; a++){
		for(b = 0; b < SUB_PIXELS; b++){
			Color	c;
			r.dir.x = x + sub_x*a;
			r.dir.y = y + sub_y*b;
			r.dir.z = -scene.view_z;
			norm_vector(&r.dir);
			
			c = ray_trace(&r, 0);
			retc.r += c.r / (float)(SUB_PIXELS*SUB_PIXELS);
			retc.g += c.g / (float)(SUB_PIXELS*SUB_PIXELS);
			retc.b += c.b / (float)(SUB_PIXELS*SUB_PIXELS);
		}
	}

	return retc;
}

/* trace rays for entire viewing window */
static void ray_trace_scene(void)
{
	int			i, j;
	struct timeval		tv_begin, tv_end;

	gettimeofday(&tv_begin, NULL);

#pragma omp parallel for schedule(dynamic)
	for(i = 0; i < win_size_y; i++){
		// print statement to help you keep track of progress
		scan_line = i;

#pragma omp parallel for schedule(dynamic)
		for(j = 0; j < win_size_x; j++){
			bitmap[i][j] = render_pixel(i, j);
		}
	}

	gettimeofday(&tv_end, NULL);

	printf("%ld us\n", 
		1000000*(tv_end.tv_sec - tv_begin.tv_sec) +
		(tv_end.tv_usec - tv_begin.tv_usec));
}


/**
 * Allocates buffers used in the ray trace
 */
static void alloc_buffers(int wsize_x, int wsize_y)
{
	int	i;

	bitmap = (Color **)malloc(sizeof(Color *) * wsize_x);
	for(i = 0; i < wsize_y; i++){
		bitmap[i] = (Color *)malloc(sizeof(Color) * wsize_y);
	}
}

static int gWindowSizeX,gWindowSizeY;

/* setup precomputed values for solids */
static void init_solids(void)
{
	int	i;
	for(i = 0; i < scene.solid_c; i++){
		scene.solids[i].f->init(scene.solids[i].obj);
	}
}

// Reshape the window and record the size so
// that we can use it in the display callback.
void ReshapeCallback(int w, int h)
{
	gWindowSizeX = w;
	gWindowSizeY = h;

	glViewport(0, 0, gWindowSizeX, gWindowSizeY);

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	gluOrtho2D(0, gWindowSizeX, 0, gWindowSizeY);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	win_size_x = gWindowSizeX;
	win_size_y = gWindowSizeY;
}


/*
*	Main function.  Reads in the scene information,
*	places it in data structures, and creates an interface window.
*/
int main(int argc, char **argv)
{
	/* initialize */
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	init_solid_funcs();

	/* read input */
	readSceneFile(argv[1]);
	alloc_buffers(win_size_x, win_size_y);
	init_solids();
	ray_trace_scene();

	/* create the interface window */
	glutInitWindowSize(win_size_x, win_size_y);
	glutInitWindowPosition(0, 0);
	glutReshapeFunc(ReshapeCallback);
	glutCreateWindow("Ray Tracing");
	gluOrtho2D(0.0, win_size_x, 0.0, win_size_y);

	/* set the display function */
	glutDisplayFunc(draw_raytrace);

	glutTimerFunc(100, update, 1);

	/* enter the main loop */
	glutMainLoop();

	return 0;
}

