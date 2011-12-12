IGM_DIR=nao_igm
SOLVER_DIR=smpc_solver


ifdef DEBUG
DEBUGFLAGS=DEBUG=1
CMAKEFLAGS=-DCMAKE_BUILD_TYPE=DEBUG
else
DEBUGFLAGS=
CMAKEFLAGS=-DCMAKE_BUILD_TYPE=Release
endif


cmake: solver igm
	-mkdir build;
ifdef TOOLCHAIN
	cd build; cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} ${CMAKEFLAGS} ..;
else
	cd build; cmake ${CMAKEFLAGS} ..;
endif
	cd build; ${MAKE}


solver:
	cd ${SOLVER_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN};

igm:
	cd ${IGM_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN};


clean: igm-clean solver-clean
	rm -f src/*.o
	rm -rf build

igm-clean:
	cd ${IGM_DIR}; ${MAKE} clean;

solver-clean:
	cd ${SOLVER_DIR}; ${MAKE} clean;


# dummy targets
.PHONY: clean

