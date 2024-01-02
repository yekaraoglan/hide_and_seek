# hide-and-seek
Main repository of the "Tom and Jerry" Project in YZV 406E

### Before Start Using the Simulation
Place all of the `.xacro` files in the `urdf` folder into `/opt/ros/noetic/share/turtlebot3_description/urdf` folder. You will need sudo permissions to do this. <br>

## Running Simulation
`roslaunch hide_and_seek multirobot_sim.launch` <br>
`roslaunch hide_and_seek multirobot_nav.launch` <br>

## Running Bots
`rosrun hide_and_seek seeker_bot.py` <br>

## Simulation World
World is designed in Gazebo to simulate a challenging map for the hide and seek game. The world can be seen below. <br>
![Photo of designed world](
    images/world.jpg
)
