/* basic data structures used for ray tracing */
#ifndef RAY_TRACE_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <GL/glut.h>

#define MAX_LIGHTS	3
#define MAX_SOLIDS	16

typedef struct{
	float	r;
	float	g;
	float	b;
} Color;

typedef struct{
	double x;
	double y;
	double z;
} Vertex;

typedef Vertex  Vector;

typedef struct{
	Vertex	origin;
	Vector	dir;	/* unit direction vector */
} Ray;

typedef struct{
	Vertex	pos;
	Color	c;
} Light;

typedef struct {
	/* returns 0 if no intersection with ray */
	/* returns # of times intersected object */
	/* vertex returns nearest intersection point */
	int	(*intersect)(void* obj, Ray* r, Vertex* v);

	/* returns normal vector to object given a vertex on the object */
	/* assumes vertex is on surface of object */
	Vector	(*normal)(void* obj, const Vertex* v);

	/* precomputes some often needed values for the solid */
	void	(*init)(void* s);
} SolidFuncs;

typedef struct{
	SolidFuncs	*f;
	void		*obj;
	Color		c;
} Solid;

typedef struct{
	Vertex	center;
	double	radius;
	double	radius_sq;	/* radius squared, used often */
} Sphere;

typedef struct{
	int	no_vertices;
	Vertex	*vertices;
	
	Vector	normal;
	float	d;	/* dot(normal, plane_vertex) + d = 0 */

	/* used for intersections */
	int	gone_coord;	/* dominate coordinate that we remove */
	float	*poly_2d[2];	/* u,v plane used for intersections */
} Poly;

typedef struct{
	float view_z;
	int light_c;
	Light lights[MAX_LIGHTS];

	int	solid_c;
	Solid	solids[MAX_SOLIDS];
} SceneInfo;

#endif
