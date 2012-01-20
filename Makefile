include ./common.mk


cmake: solver igm
	-mkdir build;
ifdef TOOLCHAIN
	cd build; cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} ${CMAKEFLAGS} ..;
else
	cd build; cmake ${CMAKEFLAGS} ..;
endif
	cd build; ${MAKE}


test: solver igm-test
	cd test; ${MAKE} ${DEBUGFLAGS}


solver:
	cd ${SOLVER_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN};

igm:
	cd ${IGM_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN};

igm-test:
	cd ${IGM_DIR}; ${MAKE} cmake ${DEBUGFLAGS} TOOLCHAIN=${TOOLCHAIN} LEG2JOINTS_ENABLE=1;

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

