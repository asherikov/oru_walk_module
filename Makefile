IGM_DIR=nao_igm
SOLVER_DIR=smpc_solver


cmake: solver igm
	-mkdir build;
ifdef TOOLCHAIN
	cd build; cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} ..;
else
	cd build; cmake ..;
endif
	cd build; ${MAKE}


solver:
	cd ${SOLVER_DIR}; ${MAKE} cmake TOOLCHAIN=${TOOLCHAIN};

igm:
	cd ${IGM_DIR}; ${MAKE} cmake TOOLCHAIN=${TOOLCHAIN};


clean: igm-clean solver-clean
	rm -f src/*.o
	rm -rf build

igm-clean:
	cd ${IGM_DIR}; ${MAKE} clean;

solver-clean:
	cd ${SOLVER_DIR}; ${MAKE} clean;


# dummy targets
.PHONY: clean

