<!-- <p align="center">
  <a href="" rel="noopener">
 <img width=200px height=200px src="https://i.imgur.com/6wj0hh6.jpg" alt="Project logo"></a>
</p> -->

<h1 align="center">ic2d_description</h1>

<div align="center">

  [![GitHub issues](https://img.shields.io/github/issues/leggedrobotics-usp/ic2d_description)](https://github.com/leggedrobotics-usp/ic2d_description/issues)
  ![GitHub pull requests](https://img.shields.io/github/issues-pr/leggedrobotics-usp/ic2d_description)
  [![GitHub forks](https://img.shields.io/github/forks/leggedrobotics-usp/ic2d_description)](https://github.com/leggedrobotics-usp/ic2d_description/network)
  [![GitHub stars](https://img.shields.io/github/stars/leggedrobotics-usp/ic2d_description)](https://github.com/leggedrobotics-usp/ic2d_description/stargazers)
  [![GitHub license](https://img.shields.io/github/license/leggedrobotics-usp/ic2d_description)](https://github.com/leggedrobotics-usp/ic2d_description/blob/main/LICENSE)

</div>

---

<p align="center"> A ROS2 description package for the IC2D platform (Impedance Control in 2 Dimensions)
    <br>
</p>

## 📝 Table of Contents
- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)
- [Feature requests](#feature_requests)
- [Contributing](#contributing)
- [Author](#author)
- [Acknowledgments](#acknowledgement)

## 🧐 About <a name = "about"></a>

This package contains the ROS2 description (URDF/xacro files) of IC2D (Impedance Control in 2 Dimensions). The files were generated using a simplified version of the CAD model available on this [other repo](https://github.com/leggedrobotics-usp/ic2d) and the [solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter). For simulation purposes, the package includes the ```virtual_spring_damper``` node, that simulates a spring-damper connection between the two moving platforms, with customizable undeformed length, stiffness and damping.

The package also has 2 launch files:

- **rsp.launch.py**: robot state publisher. Simply loads the URDF onto the ``/robot_description`` topic. Can be visualized in Rviz2 or Foxglove.

```bash
Arguments (pass arguments as '<name>:=<value>'):

    'use_sim_time':
        Use sim time if true
        (default: 'true')

    'joint_1_config':
        Joint 1 configuration. Possible values: "linmot", "hydraulic", "fixed"
        (default: 'linmot')

    'joint_2_config':
        Joint 2 configuration. Possible values: "linmot", "hydraulic", "fixed"
        (default: 'linmot')

    'link_1_mass':
        Link 1 total mass
        (default: '18.505')

    'link_2_mass':
        Link 2 total mass
        (default: '7.3822')
```

- **gz_sim.launch.py**: launches the robot in a simulated environment (Ignition Gazebo). Currently loads a simple joint position controller (from [ros2_controllers](https://github.com/ros-controls/ros2_controllers)) for preliminary testing purposes.

```bash
Arguments (pass arguments as '<name>:=<value>'):

    'undeformed_length':
        Undeformed length of the spring
        (default: '0.3')

    'stiffness':
        Stiffness of the spring
        (default: '6000.0')

    'damping':
        Damping coefficient of the damper
        (default: '100.0')

    'use_sim_time':
        Use sim time if true
        (default: 'true')

    'joint_1_config':
        Joint 1 configuration. Possible values: "linmot", "hydraulic", "fixed"
        (default: 'linmot')

    'joint_2_config':
        Joint 2 configuration. Possible values: "linmot", "hydraulic", "fixed"
        (default: 'linmot')

    'link_1_mass':
        Link 1 total mass
        (default: '18.505')

    'link_2_mass':
        Link 2 total mass
        (default: '7.3822')
```

## 🏁 Getting Started <a name = "getting_started"></a>
This repo is a standard ROS2 package (ament_cmake).

### 🛠 Prerequisites

- A ROS2 workspace (colcon)
    - See [this tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) to learn how to create your own workspace.

### 💻 Installing

As mentioned above, this repo is a standard ROS2 package. Thus, you simply have to clone it inside your workspace's **src** directory.

```bash
cd <path_to_your_ros2_ws>/src
git clone https://github.com/leggedrobotics-usp/ic2d_description.git
```

Then, use **colcon** to build.

```bash
cd <path_to_your_ros2_ws>
colcon build --symlink-install
```

## 🎈 Usage <a name="usage"></a>

Use the provided launch files to test basic robot visualization and Gazebo simulation. Parameters are listed above (generated running ```ros2 launch ic2d_description <launch_file>.launch.py --show-args```). Use the launch and URDF files layout as a starting point to load different controllers and use cases.

The ```virtual_spring_damper``` node is executed with command line arguments as follows:

```bash
ros2 run ic2d_description virtual_spring_damper undeformed_length=<value> stiffness:=<value> damping:=<value>
```

## 🔋 Feature requests <a name="feature_requests"></a>

Want something more on the robot description? Open an *Enhancement* issue describing it.

## 🤝 Contributing <a name="contributing"></a>

- Fork the repo
  - <https://github.com/leggedrobotics-usp/ic2d_description/fork>
- Check out a new branch based and name it to what you intend to do:
  - ````bash
    git checkout -b BRANCH_NAME
    ````
- Commit your changes
  - Please provide a git message that explains what you've done;
  - Commit to the forked repository.
    ````bash
    git commit -m "A short and relevant message"
    ````
- Push to the branch
  - ````bash
    git push origin BRANCH_NAME
    ````
- Make a pull request!

## ✍️ Author <a name = "author"></a>

<a href="https://github.com/Vtn21">
 <img style="border-radius: 50%;" src="https://avatars.githubusercontent.com/u/13922299?s=460&u=2e2554bb02cc92028e5cba651b04459afd3c84fd&v=4" width="100px;" alt=""/>
 <br />
 <sub><b>Victor T. N. 🤖</b></sub></a>

Made with ❤️ by [@Vtn21](https://github.com/Vtn21)

## 🎉 Acknowledgements <a name = "acknowledgement"></a>

<a href="https://github.com/evergamini">
 <img style="border-radius: 50%;" src="https://avatars.githubusercontent.com/u/142813395?v=4" width="100px;" alt=""/>
 <br />
 <sub><b>Elisa G. Vergamini</b></sub></a>

 Designer of IC2D

<!-- [![Gmail Badge](https://img.shields.io/badge/-victor.noppeney@usp.br-c14438?style=flat-square&logo=Gmail&logoColor=white&link=mailto:victor.noppeney@usp.br)](mailto:victor.noppeney@usp.br) -->

<!-- -  - Idea & Initial work -->

<!-- See also the list of [contributors](https://github.com/kylelobo/The-Documentation-Compendium/contributors) who participated in this project. -->