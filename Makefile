all: build

.PHONY: build
build: src
	colcon build --symlink-install
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

.PHONY: build_debug
build_debug: src
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

run: install
	ros2 launch simulation pippo.launch.py

debug: install
	ros2 launch simulation pippo.launch.py

clean:
	rm -rf build/ install log
