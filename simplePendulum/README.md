# Trajectory Optimisation for Pendulum using the Mini-Cheetah Motor testbed at DFKI

This folder contains multiple trajectory optimisation methods for the simple pendulum mini-cheetah motor testbed at DFKI. 

To run the python notebooks which require *Drake* upload the script to [google colab](https://colab.research.google.com/) and run it from there. The first code block of the python notebook will install *Drake* on the google colab platform and the generated trajecotry file will show up in the files tab on the left. From there, the trajectory can be downloaded and executed on the pendulum. Running *live* trajectory optimisation/stabilization is only possible if *Drake* is installed on the computer.

Further information on the trajectory optimisation methods using *Drake* can be found [here](http://underactuated.csail.mit.edu/trajopt.html).

Currently available Trajectory Opmisation Methods:

- `Direct Collocation` using Drake. File: `swingup_trajectory_dircol_drake.ipynb`.