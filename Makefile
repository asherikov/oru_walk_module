include ./common.mk


cmake: solver igm
	-mkdir build;
ifdef TOOLCHAIN
	cd build; cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} ${CMAKEFLAGS} ..;
else
	cd build; cmake ${CMAKEFLAGS} ..;
endif
	cd build; ${MAKE}


test: solver igm
	cd test; ${MAKE}


solver:
	cd ${SOLVER_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN};

igm:
	cd ${IGM_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN};


clean: igm-clean solver-clean
	cd test; ${MAKE} clean
	rm -f src/*.o
	rm -rf build
	rm -rf docs

igm-clean:
	cd ${IGM_DIR}; ${MAKE} clean;

solver-clean:
	cd ${SOLVER_DIR}; ${MAKE} clean;


# dummy targets
.PHONY: clean

