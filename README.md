# maze-solver 
## Development of a maze solver robot to navigate a provided maze without collisions and reach a specified goal.

1. Hardware Implementation

  o Arduino uno board is powered up using a 12V battery pack.  
  o A buck converter is used to step down battery volatage to be provided to the motor controller.  
  o This stepped down voltage is provided to the Motor driver circuit.  
  o The motor driver is connected to 2 motors followed by the 2 wheels.  
  o All these components are mounted on the chasis. 
  o Ultrasonic sensors are integrated for proximity sensing.
  o An infrared sensor is integrated to identify the goal at end.

2. Microcontroller Program

o Basic wall following algorithm to navigate along the path.
  
o A fine tuned PID controller to ensure a straight forward movement.
  
o Depth-First Search algorithm to navigate and solve the maze.

3. Robot Features

  o Successfully follow walls without significant errors.

  o Accurately detect and execute turns at maze corners.

  o Operates autonomously without human intervention and is stable.

  o Find the optimal path to the finish after multiple attempts.
  
