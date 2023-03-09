CC = gcc
CFLAGS = -O3 -Wall -g -Wno-unused-function -Wno-deprecated-declarations -I /opt/homebrew/Cellar/sdl2/2.26.4/include/
LDLIBS = -lm 

ifeq ($(shell uname -s), Darwin)
   LDLIBS += -framework OpenGL -framework GLUT
else
   LDLIBS += -lglut -lGLU -lGL
endif

LDLIBS += $(shell sdl2-config --libs)
LDFLAGS += $(shell sdl2-config --cflags)

tsp_src := $(wildcard tsp_*.c)
tsp_obj := $(patsubst %.c,%.o,$(tsp_src))

tsp_main:  tools.o $(tsp_obj)
test_heap: test_heap.o heap.o
a_star:    tools.o a_star.o heap.o


.PHONY: clean
clean:
	rm -f *.o
	rm -f tsp_main
	rm -f test_heap
	rm -f a_star
	rm -fr *.dSYM/
