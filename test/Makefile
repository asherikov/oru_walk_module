include ../common.mk


TESTS=\
	  test_01


all: ${TESTS}

${TESTS}:
	${CXX} ${CXXFLAGS} -c $@.cpp
	${CXX} -o $@.a $@.o ${LDFLAGS}

demo:
	${CXX} ${CXXFLAGS} -c $@.cpp
	${CXX} -o $@.a $@.o ${LDFLAGS}

clean:
	rm -f *.a *.o test_*.m

# dummy targets
.PHONY: clean
