# Eye in the Sky Maze Solver

Team Members:
- Amit Blonder, amitblon@buffalo.edu 
- Tony Costanzo, ajcostan@buffalo.edu

--- 

## Project Objective
The objective for this project is to design and implement a maze solving algorithm involving a flying quadcopter and ground-based rover drone. The quadcopter will capture imaging of a maze below it to provide the optimal solution pathing for the rover within the maze. The rover will make use of the information provided by the quadcopter to quickly and efficiently solve the maze.


## Contributions
This project is unique in the sense that it utilizes a team of robots to solve a problem. The rover and quadcopter will make use of the ROS Services feature discussed in Chapter 4 of ‘Programming Robots with ROS’ textbook to send and receive information between one another to complete the task. 


## Project Plan
In order to complete this project we intend to make use of the available resources at the `https://coex.tech/clover` website to source our Gazebo quadcopter design. We also will make use of textbook chapters 4 (Services), 9 (Building Maps of the World), and 10 (Navigating About the World). Lastly we intend to reference the Github package available at `https://github.com/IE-482-582/fall2021/tree/main/world_maker` to develop our Gazebo maze.


## Milestones/Schedule Checklist
- [x] Complete this proposal document.  *TC, Due Nov. 23* 
- [x] Clone and familiarize ourselves with the COEX clover simulation scripts *AB, Due Nov. 2*
- [x] Clone and repair the maze building code from Github *TC, Due Nov 20* 
- [x] Develop a maze in Gazebo to be solved  *TC, Due Nov. 23*
- [x] Develop edge following algorithm to keep the quadcopter within range of the maze *AB, Due Nov. 23*
- [x] Create a .launch file that includes the maze, boundaries, and both robots *TC, Due Nov. 25*
- [x] Implement maze mapping algorithm and ROS services network for transferring maze solution to rover *AB, Due Nov. 28*
- [x] Develop rover follower algorithm from provided maze solution *TC, Due Nov. 30*
- [x] Test both robots in simulation and troubleshoot as necessary *Due Dec. 2*
- [ ] Create final presentation.  *Due Dec. 4*
- [ ] Provide system documentation (README.md).  *Due Dec. 14*


## Measures of Success
- [ ] View both robot models in Gazebo
- [ ] Demonstrate that the quadcopter stays in close proximity to the maze
- [ ] Demonstrate that the quadcopter completes passes of the maze to develop an image
- [ ] Demonstrate that the rover receives information from the quadcopter and moves
- [ ] Demonstrate that the rover can successfully complete a maze
- [ ] Implement code on a real COEX quadcopter
- [ ] Have a classmate follow the steps in the README to successfully run the simulation without any help


