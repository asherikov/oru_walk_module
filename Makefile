cmake: solver igm
	-mkdir build;
ifdef TOOLCHAIN
	cd build; cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} ..;
else
	cd build; cmake ..;
endif
	cd build; ${MAKE}

solver:
	cd smpc_solver; ${MAKE} cmake TOOLCHAIN=${TOOLCHAIN};

igm:
	cd nao_igm; ${MAKE} cmake TOOLCHAIN=${TOOLCHAIN};

clean:
	rm -f src/*.o
	rm -rf build

# dummy targets
.PHONY: clean

