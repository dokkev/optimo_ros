### Environment Setup

Use [Roboligent SDK](https://bitbucket.org/roboligent_release/roboligent_sdk/src/master)'s Environment Setup & Project Setup.
After Installing the SDK, follow the steps below.

### Project Setup

1. Clone the repository and its dependencies

<!-- Clone other repository packages here that optimo_ros depends on -->

```bash
    mkdir -p ~/CODE/ros2_ws/src
    cd ~/CODE/ros2_ws/src
    git clone https://github.com/dokkev/hcrl_optimo_ros.git
```
Or, for Roboligent Bitbucket access, use the following

```
    git clone https://bitbucket.org/roboligent/optimo_ros.git
```

2. Install dependencies

```bash
    cd ~/CODE/ros2_ws
    rosdep update
    rosdep install --from-paths src/optimo_ros -y --ignore-src --rosdistro humble
```

**Note: We also recommend installing and making sure the recommended vscode extensions work.**

### Build, Test, Run

1. Build the project

```bash
    cd ~/CODE/ros2_ws
    colcon build --packages-up-to optimo_ros
    source ./install/setup.bash
```

2. Test the project

```bash
    cd ros2_ws/src/optimo_ros
    ros_packages=$(colcon list --names-only | tr '\n' ' ')
    cd ../..
    colcon test --packages-select $ros_packages
    colcon test-result --verbose
```

## Internal Development

### Versioning for Pull Requests
 
In pull requests, all packages are supposed to follow the same versions and should be bumped together. To bump the patch number of the project (all packages) use the following command:

```bash
    cd ros_ws/src/optimo_ros
    python3 ./bump_version.py patch
```

### Setup for Releases

1. Update Changelog to move unreleased changes to the released section.

2. Update package.xml and setup.py versions for all packages. (Minor Version 4.3.12 -> 4.4.0 or Major Version 4.3.12 -> 5.0.0)

    ```bash
    cd ros_ws/src/optimo_ros
    python3 ./bump_version.py [major|minor]
    ```