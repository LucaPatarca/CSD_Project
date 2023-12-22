all: build

.PHONY: build
build: src
	colcon build
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

run: install
	ros2 launch simulation sim.launch.py

clean:
	rm -rf build/ install log