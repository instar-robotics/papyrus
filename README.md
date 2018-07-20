# Papyrus

`Papyrus` is `kehops`'s GUI program to create and edit neural networks to produce Artificial
Intelligence scripts.
It uses `ROS` in order to communicate with `kehops`, sending commands, fetching data, updating
scripts, etc.

## How to install
`Papyrus` is a Qt GUI program that interfaces with ROS. It is separate from `kehops`. We designed
the program to be a `catkin` package, and thus be compiled with `catkin_make` or `cmake` directly.

### Dependencies
Here are the dependencies necessary to build and run `Papyrus`:

- Qt5.10 or above (in particular modules `Qt5::Widgets` and `Qt5::Svg`)
- CMake version 3.1.0 or above
- ROS Lunar or above

### Fetching the source
- First, it is necessary to create a `catkin` workspace:
  ```
  $> source /opt/ros/<distro>/setup.bash
  $> mkdir -p ~/workspace/catkin_ws/src
  $> cd ~/workspace/catkin_ws/src
  $> catkin_init_workspace
  ```
  This should result in a `CMakeLists.txt` symlink in `~/workspace/catkin_ws/src`.
- Second, build the workspace:
  ```
  $> cd ~/workspace/catkin_ws
  $> catkin_make
  ```
  This should produce two directories next to `src/`, which are: `build/` and `devel/`.
- Then source your workspace environment script:
  ```
  $> source devel/setup.bash
  ```
  You will need to source this file everytime you want to work with `papyrus`
- Next, it's time to fetch the source for `papyrus`:
  ```
  $> cd ~/workspace/catkin_ws/src
  $> git clone git@git.instar-robotics.com:software/NeuralNetwork/papyrus.git
  ```
  This should give you a `papyrus` directory
- Now it's time to build `papyrus`, for that, it is necessary to be in the catkin workspace (and
  **not** in the `papyrus` directory) so
  ```
  $> cd ~/workspace/catkin_ws/
  ```
- And now we have two possibilities: either compile all packages in the workspace or compile only
  `papyrus` (it's equivalent if you have only `papyrus` in your workspace):
  ```
  $> catkin_make
  ```
  will build all packages in the workspace (_make sure to run this command from the catkin
  workspace_)
  or
  ```
  $> catkin_make --pkgs papyrus
  ```
  will build only package `papyrus`
- And finally launch `papyrus` with:
  ```
  $> rosrun papyrus papyrus
  ```
  Yes you have to repeat `papyrus` twice: the first one is the name of the package, and the second
  one is the name of the executable we want to launch in the package (it happens to be the same
  here, that's all).
  Note that when typing this command, `TAB` should auto-complete the `papyrus` name. It's a good
  indicator that the environment is sourced correctly. If it does not auto-complete, chances are you
  forgot you `source devel/setup.bash`.
  
  
And you should have something like this:
![preview][1]


[1]: ./papyrus_preview.png
