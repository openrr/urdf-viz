urdf-viz
==================

Visualize [URDF(Unified Robot Description Format)](http://wiki.ros.org/urdf) file.
`urdf-viz` is written by rust-lang.

Install
--------------

### Install by `cargo`

If you are using rust-lang already and `cargo` is installed, you can install by `cargo install`.

#### Install urdf-viz

```bash
$ cargo install urdf-viz
```

#### (FYI) Install `cargo`

```bash
$ curl https://sh.rustup.rs -sSf | sh
```

and follow the instruction of the installer.


### Install by download binary from github

If you don't want to install `rust` and `cargo`, you might be able to find
binary for Ubuntu16.04 64bit [here](https://github.com/OTL/urdf-viz/releases).

For example, if you want to use v0.1.0,

```bash
$ wget https://github.com/OTL/urdf-viz/releases/download/v0.1.0/urdf-viz-0.1.0-ubuntu16.04-amd64.tar.gz
$ tar xvf urdf-viz-0.1.0-ubuntu16.04-amd64.tar.gz
$ ./urdf-viz /opt/ros/kinetic/share/pr2_description/robots/pr2.urdf.xacro
```

Command line
--------------

`urdf-viz` command will be installed by cargo.
It needs `rosrun` and `rospack` to resolve `package://` in `<mesh>` tag, and
it uses `xacro` to convert `.xacro` file into urdf file.
It means you need `$ source ~/catkin_ws/devel/setup.bash` before using
`urdf-viz`.

```bash
$ urdf-viz URDF_FILE.urdf
```

It is possible to use xacro file directly.
It will be converted by `rosrun xacro xacro` inside of `urdf-viz`.

```bash
$ urdf-viz XACRO_FILE.urdf.xacro
```

For other options, please read the output of `-h` option.

```bash
$ urdf-viz -h
```

GUI
--------------
In the GUI, you can

* Move a joint
  * set the angle of a joint by `Up`/`Down` key
  * `Ctrl` + Drag to move the angle of a joint
  * change the joint to be moved by `[` and `]`
* Inverse kinematics (only positions)
  * `Shift` + Drag to use inverse kinematics(Y and Z axis)
  * `Shift` + `Ctrl` + Drag to use inverse kinematics(X and Z axis)
  * change the move target for inverse kinematics by `,` and `.`
* `r` key to set random joints
* Move view point
  * Mouse Right Drag to translate view camera position
  * Mouse Left Drag to look around
  * Scroll to zoom in/out

Garally
--------------------

![ubr1_1.png](img/ubr1_1.png)
![ubr1_2.png](img/ubr1_2.png)
![pr2_1.png](img/pr2_1.png)
![pr2_2.png](img/pr2_2.png)
![thormang3_1.png](img/thormang3_1.png)
![thormang3_2.png](img/thormang3_2.png)

![pepper_1.png](img/pepper_1.png)
![pepper_2.png](img/pepper_2.png)
![nao_1.png](img/nao_1.png)
![nao_2.png](img/nao_2.png)

Dependencies
-------------
`urdf-viz` is strongly depend on [kiss3d](https://github.com/sebcrozet/kiss3d),
which is super easy to use, great 3D graphic engine.
