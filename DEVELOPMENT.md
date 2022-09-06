## Development and Customization

The information listed in this section may be helpful if you wish to customize the benchmarking process.

### Repository Structure

```bash
.
├── assets                  # README assets
├── docker                  # Dockerfile
├── rosbags                 # sample data
└── src                     # source code
    ├── benchmarking_suite      # benchmarking functionality
    │   ├── action                  # ROS action definition for triggering experiments
    │   ├── cfg                     # ROS dynamic_reconfigure parameter definition
    │   ├── experiments             # default output folder for experiment results
    │   ├── include                 # header files
    │   ├── launch                  # modular ROS launch files
    │   ├── msg                     # ROS message definitions
    │   ├── params                  # ROS parameter configurations
    │   └── src                     # implementation files
    ├── mqtt_bridge             # Python ROS-MQTT interface
    └── mqtt_client             # C++ ROS-MQTT interface
```

### Development Container

The default Docker image `ros-v2x-benchmarking-suite` built in [Installation](README.md#installation) is an execution image, i.e., it already contains the pre-built binaries to directly launch the benchmarking process. If you wish to modify configuration or source code, it is easier to work in a development container than having to rebuild the execution image all the time.

The development image `ros-v2x-benchmarking-suite:dev` may be built by only targeting an intermediate stage of the multi-staged Docker image build.

```bash
# ros-v2x-benchmarking-suite/
docker build -f docker/Dockerfile --target development -t ros-v2x-benchmarking-suite:dev .
```

The development image contains all required dependencies to compile the ROS workspace, but does not contain the workspace itself or pre-built executables. In order to dynamically work with the current workspace state, the ROS workspace may simply be mounted into a new container of the development image.

See below for how to launch and attach to a development container. Note that all changes in `src/` of your local repository copy are reflected into the container workspace. Remember to build and source the workspace (`catkin build && source devel/setup.bash`) on first launch. Afterwards, all `roslaunch` commands from the [Quick Start](README.md#quick-start) may simply be issued without the prepended `benchmark` alias.

#### Entering the Development Container

<details style="margin-left: 20px">
<summary><i>Cloud</i></summary>

```bash
# ros-v2x-benchmarking-suite/
docker run --rm -it \
           --volume $(pwd)/src:/root/ws/src \
           --name benchmarking-cloud-dev \
           ros-v2x-benchmarking-suite:dev
```

</details>

<details style="margin-left: 20px">
<summary><i>Vehicle</i></summary>

```bash
# ros-v2x-benchmarking-suite/
xhost +local: # allow local connections to Xserver
docker run --rm -it \
           --env DISPLAY \
           --volume /tmp/.X11-unix:/tmp/.X11-unix \
           --volume $(pwd)/rosbags:/root/ws/rosbags \
           --volume $(pwd)/src:/root/ws/src \
           --name benchmarking-vehicle-dev \
           ros-v2x-benchmarking-suite:dev
```

</details>
