
# Get all args to pass to colcon
# --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# --packages-select my_package to select a specific package

build_args := --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install --symlink-install

# build_args := --cmake-args -DCMAKE_BUILD_TYPE=Release --merge-install

extra_args :=

# See Colcon tutorial for more information:
#	https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#

build_ros2:
	@echo "Building ROS2 package..."
	@colcon build $(build_args) $(extra_args)

# See ROS2 Test tutorial for more information:
#	https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/CLI.html

test:
	@bash -c "source install/local_setup.bash && \
	export ORCHARD_CI=1 && \
	colcon test --merge-install --return-code-on-test-failure --abort-on-error \
	--event-handlers console_cohesion+ --python-testing pytest \
	--pytest-with-coverage --pytest-args --cov-report=term $(extra_args)"

test-result:
	@bash -c "source install/local_setup.bash && \
	colcon test-result --all --verbose $(extra_args)"

clean:
	@rm -rf build install log
