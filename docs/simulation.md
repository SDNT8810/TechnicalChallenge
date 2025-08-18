# Simulation

Gazebo world + Nav2 + a simple waypoint list. Robot moves, publishes `/odom` and a status topic. That’s all we need for this challenge.

Flow:
```
Gazebo -> /odom ----> logger -> Ganache
          /nav2_status -/
waypoint node -> Nav2
```

`run.sh` brings everything up. When the robot moves (or status changes) the logger pushes a tiny JSON blob as a self transaction.

Want headless? Remove the `gnome-terminal` bits and launch the processes in the same shell.
