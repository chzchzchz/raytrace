#include <assert.h>
#include <string.h>

#include "ray_trace.h"

extern SceneInfo scene;
extern SolidFuncs poly_funcs;
extern SolidFuncs sphere_funcs;

/* read a light from the file into the scene structure */
static void read_light(FILE* inputfile)
{
	int rc;

	if(scene.light_c >= MAX_LIGHTS){
		printf("too many lights specified...Exiting\n\n\n");
		exit(30);
	}
	
	rc = fscanf(inputfile, "%lg %lg %lg %f %f %f\n", 
			&scene.lights[scene.light_c].pos.x,
			&scene.lights[scene.light_c].pos.y,
			&scene.lights[scene.light_c].pos.z, 
			&scene.lights[scene.light_c].c.r,
			&scene.lights[scene.light_c].c.g, 
			&scene.lights[scene.light_c].c.b);
	assert(rc == 6);

	scene.light_c++;
}

/* read a polygon from the inputfile into the scene structure */
static void read_poly(FILE* inputfile)
{
	Poly	*poly;
	int	i, n, rc;

	if(scene.solid_c >= MAX_SOLIDS){
		printf("too many polygons specified...Exiting\n\n\n");
		exit(40);
	}
		
	rc = fscanf(inputfile, "%d", &n);
	assert(rc == 1);

	poly = malloc(sizeof(Poly));
	poly->no_vertices = n;
	poly->vertices = (Vertex *)malloc(sizeof(Vertex) * n);

	/* slurp vertices */
	for(i = 0; i < n; i++) {
		rc = fscanf(inputfile, "%lg %lg %lg", 
			&poly->vertices[i].x,
			&poly->vertices[i].y,
			&poly->vertices[i].z);
		assert(rc == 3);
	}

	rc = fscanf(inputfile, "%f %f %f\n", 
		&scene.solids[scene.solid_c].c.r,
		&scene.solids[scene.solid_c].c.g,
		&scene.solids[scene.solid_c].c.b);
	assert(rc == 3);

	scene.solids[scene.solid_c].obj = poly;
	scene.solids[scene.solid_c].f = &poly_funcs;

	scene.solid_c++;
}

/* read a sphere from the input file into the scene structure */
static void read_sphere(FILE* inputfile)
{
	Sphere	*s;
	int	rc;

	if(scene.solid_c >= MAX_SOLIDS){
		printf("too many spheres specified...Exiting\n\n\n");
		exit(50);
	}

	s = malloc(sizeof(Sphere));
		
	rc = fscanf(inputfile, "%lg %lg %lg %lg %f %f %f\n", 
		&s->center.x,
		&s->center.y,
		&s->center.z,
		&s->radius,
		&scene.solids[scene.solid_c].c.r,
		&scene.solids[scene.solid_c].c.g,
		&scene.solids[scene.solid_c].c.b);
	assert (rc == 7);

	scene.solids[scene.solid_c].obj = s;
	scene.solids[scene.solid_c].f = &sphere_funcs;

	scene.solid_c++;
}

/*
*	read the information from the scene specification file
*	and place in data structure
*/

void readSceneFile(char *filename)
{
	FILE *inputfile;
	char type[7];
	int view = 0;

	inputfile = fopen(filename, "r");
	if(inputfile == NULL){
		printf("Cannot open specified input file.  Exiting.\n");
		exit(10);
	}

	scene.solid_c = 0;
	scene.light_c = 0;

	while(fscanf(inputfile, "%s", (char*)&type) != EOF){
		if(strcmp(type, "VIEW") == 0){
			if(view == 0){
				int rc;
				rc = fscanf(inputfile, "%f\n", &scene.view_z);
				assert (rc == 1);
				view = 1;
			}
			else{
				printf("view specified twice...Exiting\n\n\n");
				exit(20);
			}
		}
		else if(strcmp(type, "LIGHT") == 0){
			read_light(inputfile);
		}
		else if(strcmp(type, "POLY") == 0){
			read_poly(inputfile);
		}
		else if(strcmp(type, "SPHERE") == 0){
			read_sphere(inputfile);
		}
	}

	fclose(inputfile);
}
