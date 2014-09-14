# README #

Open source - feel free to use completely or for inspiration for an Arduino robot.

By Anna Dunblad and Tony Larsson 2014. 
Last update April 2014

 
### What is this repository for? ###

Rasputnik is a line-following, object-avoiding and maze solving robot built for the competition Cybertech 2014 hosted by the student association Reset at the Polytechnic University of Madrid.

Rasputnik consist of a Arduino Uno R3 microcontroller board, a Arduino Motor Shield R3, two SRF05 ultrasound sensors, one Sharp GP2Y0A21YK infrared proximity sensor, one QTR-8RC Reflectance Sensor Array and encoders built by two simple CNY70 Reflective Optical Sensors. 

The code is adapted to these sensors. 

To switch between line following and maze solving, one switches cases within the code. It is quite easy to continue building on a solution where to robot looks for walls to decide whether it is inside a maze or not, and switches case accordingly. 

##Abilities of the robot:##

**Case line-following without abstacles:**

* Starting when a "flag" (paper, fabric, card board etc) is held close infront of  it and then quickly drawn away
* Follow a line, up to 100 degrees turns. 


* Chose between two alternative ways by looking for a black spot before turns to decide which way is the correst (in the competition, robots would at times be faced with two alternative ways to turn. The shortest option was marked by a dot of black tape five centimeters on the same side as the correct option before the y-turn)


**Case line-following with obstacles:** (on a track with two lanes - inner and outer- and obstacles and other robots)

* Starting when a "flag" (paper, fabric, card board etc) is held close infront of  it and then quickly drawn away

* Follow a line with turns less than 90 degrees (due to greater speed than in non-obstacles line-following)

* Switch lanes when faced with an obstacle

**Maze-solving: **

* Starting when a "flag" (paper, fabric, card board etc) is held close infront of  it and then quickly drawn away

* Solving a maze by the follow-wall principle (which wall is can be changed)

* Turning 90 degrees when turning

* Turning 180 degrees when faced with a dead-end

* Turning 180 degrees when crossing a line (a drawn line on the floor of the maze indicated in the 
competition that the way ahead lead to a dead end)


### Questions about the code? ###

Contact me on anna.dunblad@gmail.com