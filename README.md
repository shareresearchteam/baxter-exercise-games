# Baxter Exercise Games

This repository contains the source code for human-robot exercise games with the Rethink Robotics Baxter Research Robot. The eight games included in this code base (and an initial study using these activities) are further described in a journal article, currently under review.

A video explaining how to play each exercise game are available here: https://youtu.be/5zlaqlJJpts

## Files

HandGames.py: Main file for launching a selected game and starting the associated processes, such as recording data and allowing invervention using the keyboard.

Game.py: Main file containing all eight exercise games, plus a protocol for resetting the pose of the robot and for having Baxter wave hello to the user.

HandEngine.py: File setting up contact detection abilities during the exercise games, including the ability to monitor strength of hits, timing between hits, etc.

AudioManager.py: File setting up the ability to play MP3 files and songs from our custom note markup for certain exercise games.

## Songs

This folder contains an example MP3 file that can be used with the Strength Game (in the MP3 folder) and an example note markup file that can be used with the Stretch Game (in the TwoHand file).

## Launching the Games

This repostitory can be set up as a package in a ROS environment. It depends on also having the Baxter Software Development Kit installed and correctly configured. The games also require our open source Baxter Face Database, available here: https://github.com/nfitter/BaxterFaces. Provided all these things are set up corretly, you can launch a game with the following process.

Change directory to catkin_ws, for example, with:
```bash
cd catkin_ws/
```

Start Baxter shell with:
```bash
./baxter.sh
```

In a first open terminal, enable the Baxter robot with:
```bash
rosrun baxter_tools enable_robot.py -e
```

For our study, we suspended the use of sonar to lower the noise coming from the robot, since we did not need the sonar functionalty. This can be done with: 
```bash
rostopic pub -1 /robot/sonar/head_sonar/set_sonars_enabled std_msgs/UInt16 0
```

We also suspend contact detection, since in these game, it is actually intended/necessary for people to contact the robot. To do this, run the following in separate terminal tabs:
```bash
rostopic pub /robot/limb/left/suppress_contact_safety std_msgs/Empty -r 10
rostopic pub /robot/limb/right/suppress_contact_safety std_msgs/Empty -r 10
```

You can then move to the location of this repository (which should also contain the Baxter faces in a folder called 'faces'). There, you can rosrun HandGames.py and launch your desired Baxter behavior (resetarms/hello/GameA/GameB/GameC/GameD/GameE/GameF/GameG/GameH).

## License

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.
