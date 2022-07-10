.PHONY: all
all: format test build

.PHONY: format
format:
	clang-format src/*.h* src/*.c* test/*.c* -i

.PHONY: build
build:
	mkdir -p build
	cd build && \
	cmake .. && \
	make

.PHONY: test
test:
	mkdir -p build
	cd build && \
	cmake .. && \
	make &&\
	test/KALMAN_TEST

.PHONY: debug
debug:
	mkdir -p build
	cd build && \
	cmake -DCMAKE_BUILD_TYPE=debug .. && \
	make

.PHONY: clean
clean:
	rm -rf build
