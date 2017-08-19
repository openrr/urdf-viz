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
$ cargo install urdf_viz
```

#### (FYI) Install `cargo`

```bash
$ curl https://sh.rustup.rs -sSf | sh
```

### Install by download binary from github

If you don't want to install `rust` and `cargo`, you might be able to find
binary for Ubuntu16.04 64bit [here](https://github.com/OTL/urdf-viz/releases).

Install mesh converter commands (optional)
---------------------------------------------

```bash
$ sudo apt-get install meshlab assimp-utils
```


Command line
--------------

`urdf_viewer` command will be installed by cargo.
It needs `rosrun`, `rospack` to resolve `package://` in `<mesh>`, and
it uses `xacro` to convert `.xacro` file.
It means you need `$ source ~/catkin_ws/devel/setup.bash` before using
`urdf_viewer`.

```bash
$ urdf_viewer URDF_FILE.urdf
```

It is possible to use xacro file directly.
It will be converted by `rosrun xacro xacro` inside of `urdf_viewer`.

```bash
$ urdf_viewer XACRO_FILE.urdf.xacro
```

The default mesh converter is `assimp`. Sometimes it fails to create currect
meshes. (for example, `nao`, `pepper` models fails)

If you failed to convert mesh files, try `-m` option to use `meshlabserver`.
(It needs meshlab)

```bash
$ urdf_viewer -m URDF_FILE.urdf
```

For other options, please read `-h` option.

```bash
$ urdf_viewer -h
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
![pepper_1.png](img/pepper_1.png)
![pepper_2.png](img/pepper_2.png)
![nao_1.png](img/nao_1.png)
![nao_2.png](img/nao_2.png)
![thormang3_1.png](img/thormang3_1.png)
![thormang3_2.png](img/thormang3_2.png)
![pr2_1.png](img/pr2_1.png)
![pr2_2.png](img/pr2_2.png)

Dependencies
-------------
`urdf-viz` is strongly depend on [kiss3d](https://github.com/sebcrozet/kiss3d),
which is super easy to use, great 3D graphic engine.
