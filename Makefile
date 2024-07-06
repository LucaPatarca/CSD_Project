all: build-all

.PHONY: build-all
build-all: src
	colcon build
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

.PHONY: build
build: src
	colcon build --packages-select=tutorial_pkg
	@echo ''
	@echo 'REMEMBER TO RUN ". install/setup.bash"'

run: install
	ros2 launch tutorial_pkg sim.launch.py use_sim_time:=True

clean:
	rm -rf build/ install log
