all: octomap

SOURCE_DIR  = ../../src
BUILD_DIR  = ../../build

octomap: 
	cd $(BUILD_DIR) && cmake ../src
	cd $(BUILD_DIR) && make

clean:
	cd $(BUILD_DIR) && make clean
