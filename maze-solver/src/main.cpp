

#include <Arduino.h>

#define trigPinLeft 2
#define echoPinLeft 3

#define trigPinRight 11
#define echoPinRight 10

#define trigPinFront 4
#define echoPinFront 8

#define IRpin A0 // IR sensor pin 

#define ENA 5 // pins for left motor
#define IN1 6
#define IN2 7

#define ENB 9 // pins for Right motor
#define IN3 12
#define IN4 13

enum Direction {WEST = 0, SOUTH = 1, EAST = 2, NORTH = 3};
Direction currentDirection = NORTH;
Direction targetDirection;

Direction direction_order[4] = {WEST, SOUTH, EAST, NORTH};
 
bool leftWall;                   
bool rightWall;                                 
bool frontWall;

bool northWall;
bool southWall;
bool westWall;
bool eastWall;
                                               
long distanceFront;                            
long distanceLeft;
long distanceRight;

int baseSpeed = 145;

double error, derivative, correction, previous = 0;

struct Cell { // store the coordinates of the matrix
  int x; // X-coordinate
  int y; // Y-coordinate
};

struct Queue{ // Define a queue data type to store thw stacks.
  Cell data[9*9];
  int top = -1;

  void push(Cell cell){ //append a cell to the queue (Last element)
    data[++top] = cell;
  }

  Cell pop(){ // pop out the last element from the queue
    return data[top--];
  }

  bool isEmpty(){ // Return if the queue is empty or not 
    return top == -1;
  }

  int size() { // Return how many elements are in the array
    return top + 1;
  }
};

bool cell_wall[4];
Queue Frontier;
Queue Explored;
Queue DeadEnds;
Cell child_cell;
Cell goal_cell;
int Explored_size;

int kp = 4;  // Proportional gain
int kd = 0.5 ;  // Derivative gain

long readUltrasonic(int trigPin, int echoPin) {// Function to read distance from an ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * 0.0343) / 2;  // Calculate distance in cm
}

void DirChange_right() { // Moves the direction counterclockwise. Adding 3 achieves a backward rotation in modular arithmetic: NORTH -> EAST EAST -> SOUTH SOUTH -> WEST WEST -> NORTH
  currentDirection = static_cast<Direction>((currentDirection + 3) % 4);
}

void DirChange_left() { //  Moves the direction counterclockwise. Adding 3 achieves a backward rotation in modular arithmetic: NORTH -> WEST WEST -> SOUTH SOUTH -> EAST EAST -> NORTH
  currentDirection = static_cast<Direction>((currentDirection + 1) % 4);
}

void DirChange_180() {  // Reverse the current direction. Example: NORTH -> SOUTH
  currentDirection = static_cast<Direction>((currentDirection + 2) % 4);
}

void getSensorState(){ // Get the current readings from the sensors
  distanceFront = readUltrasonic(trigPinFront, echoPinFront);
  distanceLeft = readUltrasonic(trigPinLeft, echoPinLeft);
  distanceRight = readUltrasonic(trigPinRight, echoPinRight); 

  if(distanceLeft <= 15){
    leftWall = true;
  }else{
    leftWall = false;
  }

  if(distanceRight <= 15){
    rightWall = true;
  }else{
    rightWall = false;
  }

  if(distanceFront <= 15){
    frontWall = true;
  }else{
    frontWall = false;
  }
}

void interpretSensorReadings() {  // Map the relavent sensors to global directions 
  getSensorState();

  switch (currentDirection) {
    case NORTH:
      northWall = frontWall;
      westWall = leftWall;
      eastWall = rightWall;
      southWall = true;
      break;
    case EAST:
      eastWall = frontWall;
      northWall = leftWall;
      southWall = rightWall;
      westWall = true;
      break;
    case SOUTH:
      southWall = frontWall;
      eastWall = leftWall;
      westWall = rightWall;
      northWall = true;
      break;
    case WEST:
      westWall = frontWall;
      southWall = leftWall;
      northWall = rightWall;
      eastWall = true;
      break;
  }
  // define a cell_wall array to hold the wall configurations in a particular cell
  cell_wall[WEST] = westWall;
  cell_wall[SOUTH] = southWall;
  cell_wall[EAST] = eastWall;
  cell_wall[NORTH] = northWall;

  // Serial.println(cell_wall[0]);
  // Serial.println(cell_wall[1]);
  // Serial.println(cell_wall[2]);
  // Serial.println(cell_wall[3]);
}

bool IsInExplored(Cell child){ // Chech whether a given cell is in the explored stack
  Explored_size = Explored.size();

  for (int i = 0; i < Explored_size; i++) { //Loop through alla the elements in the explored stack and check the coordinates 
      if (Explored.data[i].x == child.x && Explored.data[i].y == child.y) {
          return true;  // Cell found in the explored list
      }
  }
  return false;
}

void turnRight_motorCommands(){ // Command the motors to turn Right
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 170);
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 170);
}

void turnLeft_motorCommands(){ // Command the motors to turn Left
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 170);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 170);
}

void stopMotor_motorCommands() { // Completely stop the motors 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void RotateRight(){ // Turn the robot 90 degrees to the right
  if ((readUltrasonic(trigPinFront, echoPinFront)) < 15){ // check whether a forward wall is available 
    stopMotor_motorCommands();
    delay(250);
    turnRight_motorCommands();
    while(readUltrasonic(trigPinFront, echoPinFront) <= 25){ // Turn right untill the front sensor will not detect a wall.
      delay(30);
    }
    delay(70);
    stopMotor_motorCommands();
    delay(250);
  }
  else{ // right turn with no wall available
    stopMotor_motorCommands();
    delay(250);
    turnRight_motorCommands();
    delay(300);
    stopMotor_motorCommands();
    delay(250);
  }
}

void RotateLeft (){ //Turn the robot 90 degrees to the right
  if ((readUltrasonic(trigPinFront, echoPinFront)) < 15){ // check whether a forward wall is available 
    stopMotor_motorCommands();
    delay(250);
    turnLeft_motorCommands();
    while(readUltrasonic(trigPinFront, echoPinFront) <= 25){ // Turn right untill the front sensor will not detect a wall.
      delay(10);
    }
    delay(50);
    stopMotor_motorCommands();
    delay(250);
  }
  else { // left turn with no wall available
    stopMotor_motorCommands();
    delay(250);
    turnLeft_motorCommands();
    delay(300);
    stopMotor_motorCommands();
    delay(250);
  }
}

void Rotate180(){ //Turn the robot 180 degrees
  if ((readUltrasonic(trigPinRight, echoPinRight)) < 10){  // check whether right wall is available and then 
                                                          // robot will turn 180 degrees by taking the feedback from right wall
    stopMotor_motorCommands();
    delay(250);
    turnRight_motorCommands();
    while(readUltrasonic(trigPinFront, echoPinFront) <= 25){
      delay(50);
    }
    stopMotor_motorCommands();
    delay(250);

  }else if((readUltrasonic(trigPinLeft, echoPinLeft)) < 10){
    stopMotor_motorCommands();
    delay(250);
    turnLeft_motorCommands();
    while(readUltrasonic(trigPinFront, echoPinFront) <= 25){
      delay(50);
    }
    stopMotor_motorCommands();
    delay(250);
  }
}

void leftwallForward(){
  int kp = 3;  // Proportional gain
  int kd = 1.5 ;  // Derivative gain

  int targetDistance = 5;

  // Calculate the error between left and right distances
  double errorLeft = targetDistance - readUltrasonic(trigPinLeft, echoPinLeft);

  // Calculate the derivative (rate of change of error)
  double derivative = (errorLeft - previous) / 0.05;

  // Apply PD control: combine proportional and derivative terms
  correction = (kp * errorLeft) + (kd * derivative);

  // Update the previous error for the next loop
  previous = errorLeft;

  // Adjust motor speeds with correction
  int speedA = baseSpeed + correction;
  int speedB = baseSpeed - correction;

  // Ensure the speeds are within the valid range
  speedA = constrain(speedA, 140, 170);
  speedB = constrain(speedB, 140, 170);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speedA);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedB);
}

void rightwallForward(){
  int kp = 7;  // Proportional gain
  int kd = 1.75 ;  // Derivative gain

  int targetDistance = 6;

  // Calculate the error between left and right distances
  double errorRight = targetDistance - readUltrasonic(trigPinRight, echoPinRight);

  // Calculate the derivative (rate of change of error)
  double derivative = (errorRight - previous) / 0.05;

  // Apply PD control: combine proportional and derivative terms
  correction = (kp * errorRight) + (kd * derivative);

  // Update the previous error for the next loop
  previous = errorRight;

  // Adjust motor speeds with correction
  int speedA = baseSpeed - correction;
  int speedB = baseSpeed + correction;

  // Ensure the speeds are within the valid range
  speedA = constrain(speedA, 140, 200);
  speedB = constrain(speedB, 140, 200);
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, (speedA+5));
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speedB);
}

void moveForward2(){ // Move forward without following a wall

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, 164);
  analogWrite(ENB, 155);

}

void moveStep(){ // Go forward for one step (one tile)

  unsigned long starttime = millis(); 
  const unsigned long moveDuration = 830;

  while ((millis() - starttime) < moveDuration) {
    if ((readUltrasonic(trigPinFront,echoPinFront)) < 10){
      break;
    }

    if (readUltrasonic(trigPinRight, echoPinRight) < 20) {
      rightwallForward();
    }
    else{
      moveForward2();
    }
    delay(10);
  }
  stopMotor_motorCommands();
  delay(500);

  // moveForward2();
  // delay(830);
  // stopMotor_motorCommands();
  // delay(250);
  // stopMotor_motorCommands();
  // delay(500);
}


void moveRobot(Direction target, Direction current) { //when given the current direction and the target direction this will realize which rotations it should take
    // Determine the rotation needed
    int rotation = (target - current + 4) % 4;

    // Rotate the robot based on the difference
    switch (rotation) {
        case 0: // No rotation needed
            break;
        case 1: // Rotate 90 degrees right
            RotateLeft();
            DirChange_left();
            break;
        case 2: // Rotate 180 degrees
            Rotate180();
            DirChange_180();
            break;
        case 3: // Rotate 90 degrees left
            RotateRight();
            DirChange_right();
            break;
    }

    // Move forward one tile
    moveStep();
    delay(250);

    // Update current direction
    currentDirection = targetDirection;
}

void getTargetDirection (int x, int y){ // Based on the current and desired cells, we can determine which direction robot should travel
  if (x == 0 && y == 1) {
    // Move North
    targetDirection = NORTH;

  } else if (x == 0 && y == -1) {
    // Move South
    targetDirection = SOUTH;

  } else if (x == 1 && y == 0) {
    // Move East
    targetDirection = EAST;

  } else if (x == -1 && y == 0) {
    // Move West
    targetDirection = WEST;
  }
}

bool readIRsensor(){ // Detect the white color cell using IR sensor reading
  int detectionThreshold = 400;
  int sensorVal = analogRead(IRpin);
  // Serial.println(sensorVal);
  // delay(500);
  if (sensorVal > detectionThreshold){
    return false;
  }else{
    return true;
  }
}

void DFS(Cell start){ //DFS code
  // initially start cell is pushed to both frontier and explored stacks.
  Frontier.push(start); 
  Explored.push(start);
  Cell previous_cell = start; 

  while (!Frontier.isEmpty()){ // Main while loop. This will run untill there are no elements in frontier stack
    Cell current_cell = Frontier.pop();

    if ((current_cell.x == start.x) && (current_cell.y == start.y)){ // because at start we place the robot in that cell 
      previous_cell = current_cell;
      Explored.push(current_cell);
    }
    else{
      // Determine the direction to move based on the difference between cell coordinates
      int deltaX = current_cell.x - previous_cell.x;
      int deltaY = current_cell.y - previous_cell.y;

      getTargetDirection(deltaX, deltaY);

      //Based on the target and current distace we can determine the direction we need to move 
      moveRobot(targetDirection, currentDirection);

      delay(100);
      // After we move the robot we push the current cell to explored list 
      Explored.push(current_cell);
      previous_cell = current_cell; // keep track of the previous cell 
    }

    int detectionThreshold = 400;
    int sensorVal = analogRead(IRpin);
    // Serial.println(sensorVal);
    // delay(500);
    if (sensorVal < detectionThreshold){
      delay(100);
      break;
    }
    
    int validChildCount = 0;

    interpretSensorReadings(); // Get all the sensor readings for current cell and create the cell_wall array

    for (int i = 0; i < 4; i++){ // i represent the direction {0 = WEST, 1 = SOUTH, 2 = EAST, 3 = NORTH}
      // iterate through each direction to find child cells 
      Direction d = direction_order[i];

      if (cell_wall[i] == false){ //
      // if a particular cell is traversable (No wall in that direction) and not in the explored list iit is considered as a child cell
        if (d == WEST){
          child_cell.x = current_cell.x - 1; 
          child_cell.y = current_cell.y;
        }

        else if (d == SOUTH){
          child_cell.x = current_cell.x; 
          child_cell.y = current_cell.y - 1;
        }  

        else if (d == EAST){
          child_cell.x = current_cell.x + 1; 
          child_cell.y = current_cell.y;
        } 

        else if (d == NORTH){
          child_cell.x = current_cell.x; 
          child_cell.y = current_cell.y + 1;
        }

        if (IsInExplored(child_cell)) {
          continue;
        }

        validChildCount++; // keep track of child cells 
        Frontier.push(child_cell); // every child cells are pushed into frontier array in reverse priority order.
      }
    }
    if (validChildCount == 0){ // If there is no valid child cells, it is the goal or dead end. Goal is checked above so this is a dead end
      Serial.println("dead end reached");
      bool breakAll = false;
      for (int i = Explored.size() - 1; i >= 0; i-- ){ // backtrack through explored cells to find a cell that has a unvisited child cell
        Cell target_cell = Explored.data[i-1];

        int deltaX = target_cell.x - current_cell.x;
        int deltaY = target_cell.y - current_cell.y;

        getTargetDirection(deltaX, deltaY);

        Serial.println("moving back");

        moveRobot(targetDirection, currentDirection);

        DeadEnds.push(Explored.pop());

        current_cell = target_cell;

        Cell neighbour_cell;
        delay(1000);

        for (int i = 0; i < 4; i++){  // check all the neighbour cells of current cell in all directions 
          Direction d = direction_order[i];

          interpretSensorReadings();

          if (cell_wall[i] == false){ // neighboure cells are the ones that can travel from the current cell (traversable cells)
            if (d == NORTH){
              neighbour_cell.x = current_cell.x; 
              neighbour_cell.y = current_cell.y + 1;
            }

            else if (d == EAST){
              neighbour_cell.x = current_cell.x + 1; 
              neighbour_cell.y = current_cell.y;
            } 

            else if (d == WEST){
              neighbour_cell.x = current_cell.x - 1; 
              neighbour_cell.y = current_cell.y;
            } 

            else if (d == SOUTH){
              neighbour_cell.x = current_cell.x; 
              neighbour_cell.y = current_cell.y - 1;
            }

            if ((neighbour_cell.x == (Frontier.data[Frontier.size()-1].x)) && (neighbour_cell.y == Frontier.data[Frontier.size()-1].y)){
              // If there is a neighboure cell that has a unvisited child cell we break the loop and go to the main while loop again 
              breakAll = true;  
              break;
            }
          }
        }
        previous_cell = current_cell; // track the current cell 
        if (breakAll) break;

      }
    }
  }
}

void setup() {
  pinMode(ENA, OUTPUT);   // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);

  pinMode(trigPinLeft, OUTPUT);  //Ultrasonic sensor pins setup
  pinMode(echoPinLeft, INPUT);

  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);

  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
}

void loop() {
  Cell start; //initiate the starting cell as (0,0)
  start.x = 0;
  start.y = 0;

  DFS(start);

  // readIRsensor();
  // leftwallForward();
  // rightwallForward();
  // moveForward2();
  delay(10);
}