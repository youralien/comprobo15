#Warmup Project Writeup    
by Zhecan Wang and Ryan Louie
-----------------------------

##Which behaviors did you implement?
- Wall following
- Person following
- Object avoidance

##For each behavior, what strategy did you use to implement the behavior? 
- Wall following
    - bump sensors were used exclusively
    - the bump sensors help the Neato orient itself parallel to the wall; if the left sensor was bumped, it would reverse left, or vice versa. Then, it would attempt to go forward again.
    - this process would continue until it achieved a parallel state with the wall; the Neato would then move forward until its bumps sensors are triggered again.
- Person following
    - laser scan was used, with a set forward "cone" 1) range and 2) width or angle.
    - Sensor readings are measured for the left and right side of the cone; the collective left and right range measurements were averaged together.
    - If the average-left-range-readings was less than the right, this meant that the person was located left of the robot, or vice versa.
    - The difference between the left and right readings determined how far off centered the Neato was from the person it should follow.
    - At the same time, a distance measure was also taken across the "cone" of scans. We used proportional control to achieve a particular distance away from the person.
- Object avoidance
    - laser scan was used, sensing the front left/right quadrants of the Neato.
    - all the while, the robot was instructed to move a unit distance forward before checking the laser scan readings.
    - if an object was in front of it, it would enter a "turn" mode. It determined which quadrant the object was in.  If left quadrant, the Neato would turn 90 degrees to the right.
    - a flag would be set telling the robot that it needed to adjust its angle back to the original orientation, once it is able.
    - the Neato would move a unit distance, side stepping the object. If the object was still in sight, it would move again, side stepping the object even more. Finally, when the object is no longer in view, it would start the reorientation to the original angle.
    - Once the original angle is achieved, it would attempt to move towards the original bearing, one unit distance at a time, checking to make sure no obstacles are in the way.

##For the finite state controller, what were the states?  What did the robot do in each state?  How did you combine and how did you detect when to transition between behaviors?

##How did you structure your code?
- Much of the sensing and acting happens together in the callback method.
- Some helper methods that are called in many situations are factored out into helper methods of the class. A few good examples is from ObstacleAvoidance.adjust_original_angle() or ObstacleAvoidance.params_to_go_forward().
- All scripts are object oriented, with common __init__, callback, and run methods.

##What if any challenges did you face along the way?
- We faced many logic difficulties. The complexity of callbacks depending on changing instance variables was easy to fool us in thinking we understood every nuance of the logic.

##What would you do to improve your project if you had more time?
Ryan: I would like to try to make the obstacle avoidance robot based on the potential fields approach. Seeing robot motion in a fluid movement is a lot more satisfying than it navigating as if it was on a grid of squares (making "L" motions only).

##Did you learn any interesting lessons for future robotic programming projects?
- Never start with writing out the thoughts of your entire algorithm before testing its component parts. Here are a few reasons this is bad!
    1. It is very unlikely that each of the parts will be integrated perfectly
    2. It is difficult to debug which component part is broken
    3. You can think of this full, untested implementation as a "bad initialization" of your code. It will take much time to debug this bad initialization, rather than starting with good, small code chunks to begin with.
- If you do feel compelled to think about the entire algorithm, write pseudo code, which may inspire you to make a backlog of component parts.  Make sure the robotic sensors and responses work as you expect before placing this component parts in the logic structure of your algorithm.

