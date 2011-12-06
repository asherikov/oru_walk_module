cmake: 
	-mkdir build;
ifdef TOOLCHAIN
	cd build; cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN} ..;
else
	cd build; cmake ..;
endif
	cd build; ${MAKE}


clean:
	rm -f src/*.o
	rm -rf build

# dummy targets
.PHONY: clean

