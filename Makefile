ray: ray_trace.o read_input.o
	gcc $^  -lgomp -lglut -lGLU -lGL -lm -o $@

clean:
	rm -rf *.o ray

ray_trace.o: ray_trace.c ray_trace.h
	gcc -Wall -fopenmp -O2 ray_trace.c -c -o ray_trace.o

read_input.o: read_input.c
	gcc -Wall -O2 read_input.c -c -o read_input.o
