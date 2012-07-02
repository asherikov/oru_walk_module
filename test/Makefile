include ../common.mk


CXXFLAGS += -I/usr/local/include/eigen2/ -I/usr/include/eigen2/


TESTS=\
	test_04 \
	test_07 \
	test_08 \
	test_09 \
	test_10

TESTS_GL=\
	test_05 \
	test_06


all: ${TESTS}

${TESTS}:
	${CXX} ${CXXFLAGS} -c $@.cpp
	${CXX} -o $@.a $@.o ${LDFLAGS}


${TESTS_GL}: glflags
	${CXX} ${CXXFLAGS_GL} -c $@.cpp
	${CXX} -o $@.a $@.o ${LDFLAGS_GL}


glflags: 
SYSTYPE = $(shell uname -s)
CXXFLAGS_GL = ${CXXFLAGS}
LDFLAGS_GL = ${LDFLAGS} -lleg2joints
ifeq ($(SYSTYPE), Linux)
CXXFLAGS_GL += -I/usr/X11R6/include `sdl-config --cflags` -DLINUX
LDFLAGS_GL += -L/usr/X11R6/lib -lGL -lGLU -lm -lpng -lz `sdl-config --libs`
endif
ifeq ($(SYSTYPE), FreeBSD)
CXXFLAGS_GL += -I/usr/local/include `sdl-config --cflags` -DLINUX
LDFLAGS_GL += -L/usr/local/lib -lGL -lGLU -lm -lpng -lz `sdl-config --libs`
endif
ifeq ($(SYSTYPE), Darwin)
CXXFLAGS_GL += -I/usr/local/include/SDL -D_GNU_SOURCE=1 -D_THREAD_SAFE -DDARWIN
LDFLAGS_GL += -L/usr/local/lib -lSDLmain -lSDL -Wl,-framework,Cocoa -framework OpenGL
endif


clean:
	rm -f *.a *.o test_*.m

# dummy targets
.PHONY: clean
