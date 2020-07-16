# Introduction #

Hello, welcome to Automated Infinite Recharge! This is a Python project written by FRC Team 3260. We're attempting to
write a 100% completely autonomous robot to play the 2020 FRC game, called Infinite Recharge, using the [FRC 2020
Simulator](https://github.com/ptkinvent/frcsim2020). We're leveraging the power of a simulated LIDAR sensor in the sim.

We intentionally chose not to pursue a deep-learned approach to our robot because we preferred a fully deterministic
stack. Not only is this easier to debug, but we were also exposed to a greater diversity of algorithms this way.

At a high level, our stack consists of three components: perception, planning, and controls. Data travels sequentially
through each of these steps, starting with raw LIDAR and other sensor data coming into the perception stack. The final
output from controls consists of motor signals for the tank drive and ball intake/outtake system.

![A high-level overview of our autonomy stack](img/autonomy-stack.svg)
