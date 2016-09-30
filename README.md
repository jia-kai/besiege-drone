# Besiege Drone

A drone in Besiege which can clear almost all the levels.

[![argus-demo](https://j.gifs.com/2k5ZYv.gif)](https://youtu.be/fyW_sBMxHPQ)

## Usage

* Run `./controller.py`, which would start a socket server for controlling the drone
* Copy `drone.bsg` and `drone_lench_client.py` into
  `Steam/steamapps/common/Besiege/Besiege_Data/SavedMachines`, and use
  [LenchScripterMod](https://github.com/lench4991/LenchScripterMod) to load the
  `drone_lench_client.py`.
* Control:

    * Keyboard for moving; see `actiondef.py` for keymap.
    * Mouse click to set a target hover position.

## How it works

The speed of four engines is controlled by the classical PID algorithm. And I
also use `rot_quaternion.py` to modify the rotation of engine blocks, so it can
be tilted for a tiny angle (3 degrees), which eases yaw controlling.
