
# Autonomous Systems
This directory houses the alorithms for the A* and point-to-point algorithms as well as explains all the algorithms implemented by the team.

# Building

To rebuild the code in this directory with colcon, we haven't found an effective way for ROS to recognize that the package must be rebuilt (it is already specificed as a package to be built in setup.py, but won't build again if already built), thus, you will need to remove the current file you would like to build in the build directory:
`sailbot20-21/sailbot_ws/build/sailbot/build/lib/sailbot/autonomous`

# Algorithms Explainations

A general flow for the algorithms is as follows: the ballast and A* run concurrently, where the P2P runs as a part of the A*. In other words, the A* acts at a high level, where the P2P works off of its decisions. After determining a path, the A* feeds its output point by point into the P2P, which begins anew for each point.

## A Star (A*)
The A* algorithm determines the waypoints that the boat should sail through. The algorithm takes a grid that is generated through the Telemetry interface. The grid is generated from a json file. The class parses the json file and creates the relevant objects to run A*. The algorithm must still be integrated into the actual code to use the waypoints for P2P. The way the class works is through this:

         Heuristic:
           - node weight: will be total distance from current node to the end
           - edge weight: will be current direction of travel + current wind direction vs edge direction
        
         Algo structure:
           - F(n) = g(n) + h(n)
               - g(n) = edge weight
               - h(n) = dest node weight
        
          - Evaluate all pathsToCheck from a node, then start continuing on the lowest cost one
          - once you find the final node, keep going through your list until the value you have is lower than
               the top of the list

For testing purposes the file can just be run on its own, and an interface will pop up on windows. Through this the algorithm can be tweaked and tested so the correct waypoints are generated.

## Point to Point (P2P)
The P2P (point to point) algorithm is used to travel between waypoints set by the A* algorithm. It takes the GPS coordinates of a desired destination and data provided by the Airmar and uses them to determine how to adjust the sail and rudders. It is controlled primarily through the getAction function, which is what you will call to update the sail and rudders. It is intended to be called repetedly to ensure the boat stays on course. The code switches among activating 3 states depending on what its state variable is set to. The first state controls the trim tab and recognizes whether or not the destination coordinates fall within the no sail zone of the boat's current location. The second state activates if the destination is within the no sail zone and instructs the rudders to follow a course that will take the boat as far upwind as possible. The third state activates if the destination is not within the no sail zone and instructs the rudders to follow a course that leads straight to the destination.

NOTE: because computer vision and collision avoidance are not yet incorporated, the boat will go directly toward its destination regardles of what is in the way. Once computer vision is added you will need to add code to the P2P algorithm to operate the rudders such that the boat doesn't crash, perhaps even a new state. Additionally, the second state instructs the boat to try to reach a destination in the no sail zone by only tacking once. When this is not possible, such as when the boat is trying to sail up a narrow chanel, you will need to instruct the boat to tack before it collides with anything. This will also require computer vision.

## Ballast
The ballast algorithm is located in the control_system node and is contained within the ballastAlgorithm() function. It works by first determining which direction the wind is coming from so that it can move the ballast to roll leeward (away from the wind). Then it checks the current roll angle provided by the airmar in the airmar_data["roll"] field and moves the ballast outward if the current roll angle is less than ideal and inward if the boat is heeling past a given angle. 

NOTE: the ideal angle of heel is approximately 20 degrees with a maximum of 25 degrees but this caused an issue where the ballast would attempt to go outside its limit. Without hall effect sensors implemented there is currently no way to know when the ballast has reached its limit and thus the target angle was reduced to 12 degrees with a maximum of 20 degrees to prevent damaging the gears and motor. Ideally more weight could be added to the ballast to acheive a proper roll, but without limiting this is unsafe.
