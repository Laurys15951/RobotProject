// File:          EPuckShortestPathRightCorners.java
// Date:
// Description:
// Author:
// Modifications:

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Camera;
import java.util.*;
import java.io.*;


// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class EPuckShortestPathRightCorners {

  //number of vertices
  static final int V = 64;
  
  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  public static void main(String[] args) {
  
    int TIME_STEP = 64;
    double MAX_SPEED = 6.28;

    // create the Robot instance.
    Robot robot = new Robot();

    //get distance sensors
    DistanceSensor[] ps = new DistanceSensor[8];
    String[] psNames = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
    };
    
    for (int i = 0; i < 8; i++) {
      ps[i] = robot.getDistanceSensor(psNames[i]);
      ps[i].enable(TIME_STEP);
    }
    
    //get camera
    Camera camera = robot.getCamera("camera");
    camera.enable(TIME_STEP);

    
    // get the motor devices
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // set the target position of the motors
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);

    var left_ps = robot.getPositionSensor("left wheel sensor");
    left_ps.enable(TIME_STEP);
    var right_ps = robot.getPositionSensor("right wheel sensor");
    right_ps.enable(TIME_STEP);
 
    //parameters to keep track of robot
    double[] wheelValues = {0,0};
    double[] lastWheelValues = {0,0};
    double[] distTraveled = {0,0};

    //robot parameters
    double wheelRadius = 0.0205;
    double distBetweenWheels = 0.052;
    double wheelCirum = 2*Math.PI*wheelRadius;
    double encoderUnit = wheelCirum/6.28;
   
    //robot pose
    double[] robotPose={0,0,0};//x,y,theta
      
    //variables for counting return direction
    double retDir;
    double retDirEpsilon = 0.02;
    
    int closeToObsticle = 0;
    boolean done = false;


    //sizes to move a square forward or turn 90 degrees
    double forward = 6.24;
    double turn = 2.4;
     
    //start and resting squares -------------------------------------------
    int start = 17;
    int finish = 6;
    
    //Epsilon for return
    double returnPointEpsilon = 0.03;
    
    //velocity
    double motorVelocity = 0.1;

   //path algorithm
    //Map graph
    int graph[][]=readData();
    //distances array
    int[] minRoads =new int[64];
    for(int i = 0; i< 64; i++){
      minRoads[i] = -1;
    }
    minRoads[start] = -2;
    
    //computing shortest road with dijkstra
    ArrayList<Integer> road = dijkstra(graph, start, finish,minRoads);
    
    //logic variables for moving forward
    int botDir = 0;
    int step = 0;
    boolean turning = false;
    boolean forwarding = false;
    double savedPose=0;
    double[] savedDistTraveled = {0,0};
    double errorInDirection = 0.02;
    double errorInStepForward = 0.001;
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(TIME_STEP) != -1) {
      //Getting new wheel possition
      wheelValues[0] = left_ps.getValue();
      wheelValues[1] = right_ps.getValue();


      // processing how musch wheels moved
      for(int i = 0; i < 2; i++){
        double diff = wheelValues[i] - lastWheelValues[i];
        if(Math.abs(diff) < 0.001){
          diff = 0;
          wheelValues[i] = lastWheelValues[i];
        }
        distTraveled[i] = diff * encoderUnit;
      }


      //compute linear and angular velocity from how much wheels moved
      double v=(distTraveled[0]+distTraveled[1])/2.0;
      double w=(distTraveled[0]-distTraveled[1])/distBetweenWheels;
      
      double dt = 1;
      robotPose[2] += (w*dt);
      
      double vx = v*Math.cos(robotPose[2]);
      double vy = v*Math.sin(robotPose[2]);
      
      robotPose[0] += (vx*dt);
      robotPose[1] += (vy*dt);
      
      //System.out.println(robotPose[0]+" "+robotPose[1]+" "+robotPose[2]);

      // logic for moving towards rest spot acording to dijkstra commads
      if(turning){
        //System.out.println(Math.PI/2+" "+robotPose[2]);
        if((savedPose+(0.17+Math.PI/2)+errorInDirection>robotPose[2]&&
            savedPose+(0.17+Math.PI/2)-errorInDirection<robotPose[2])||
            (savedPose-(0.17+Math.PI/2)+errorInDirection>robotPose[2]&&
            savedPose-(0.17+Math.PI/2)-errorInDirection<robotPose[2])){
          turning = false;
          leftMotor.setVelocity(motorVelocity * MAX_SPEED);
          rightMotor.setVelocity(motorVelocity * MAX_SPEED);
          savedDistTraveled[0] = robotPose[0];
          savedDistTraveled[1] = robotPose[1];
          //System.out.println("Turn Done");
          }
      } else if(forwarding){
        //System.out.println(robotPose[0]+" "+robotPose[1]);        
        if((savedDistTraveled[0]+0.125+errorInStepForward>robotPose[0]&&
           savedDistTraveled[0]+0.125-errorInStepForward<robotPose[0])||
           (savedDistTraveled[1]+0.125+errorInStepForward>robotPose[1]&&
           savedDistTraveled[1]+0.125-errorInStepForward<robotPose[1])||
           (savedDistTraveled[0]-0.125+errorInStepForward>robotPose[0]&&
           savedDistTraveled[0]-0.125-errorInStepForward<robotPose[0])||
           (savedDistTraveled[1]-0.125+errorInStepForward>robotPose[1]&&
           savedDistTraveled[1]-0.125-errorInStepForward<robotPose[1])){
          leftMotor.setVelocity(0);
          rightMotor.setVelocity(0);
          forwarding = false;
          //System.out.println("Forward Done");
        }
      }else {
        if(step+1 == road.size()){
          //System.out.println("Dijkstra movement done");
          break;
        }
        int stepDif = road.get(step+1)-road.get(step);
        savedPose=robotPose[2];
        savedDistTraveled[0] = robotPose[0];
        savedDistTraveled[1] = robotPose[1];
  
        //System.out.println(stepDif+" "+botDir);
        if(stepDif==1){
          if(botDir==0){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
          }else if(botDir==1){
            leftMotor.setVelocity(-motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }else if(botDir==3){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(-motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }
          botDir = 0;
        }else if(stepDif==8){
          if(botDir==1){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
          }else if(botDir==2){
            leftMotor.setVelocity(-motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }else if(botDir==0){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(-motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
           }
           botDir = 1;
        }else if(stepDif==-1){
          if(botDir==2){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
          }else if(botDir==3){
            leftMotor.setVelocity(-motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }else if(botDir==1){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(-motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }              
          botDir = 2;
        }else if(stepDif==-8){
          if(botDir==3){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
          }else if(botDir==0){
            leftMotor.setVelocity(-motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }else if(botDir==2){
            leftMotor.setVelocity(motorVelocity * MAX_SPEED);
            rightMotor.setVelocity(-motorVelocity * MAX_SPEED);
            forwarding = true;
            turning = true;
          }
          botDir = 3;
        }
        step++;
      }
      for(int i = 0; i < 2; i++){
        lastWheelValues[i] = wheelValues[i];
      }
    };
    //waiting for ~10 seconds------------------------------------------------
    //*
    int breaktime =0;
    while (robot.step(TIME_STEP) != -1) {
      if(breaktime>TIME_STEP*2.5)
        break;
      breaktime++;
    };//*/

   
    // stopping motors temporarily.
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);

    //returning to starting point
    while (robot.step(TIME_STEP) != -1) {
      // Read the position sensors:
      wheelValues[0] = left_ps.getValue();
      wheelValues[1] = right_ps.getValue();
      
      //getting distance sensors values
      double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i < 8 ; i++){
        psValues[i] = ps[i].getValue();
        //System.out.print(psValues[i]+" ");
      }
      //System.out.println();
      
      // processing how musch wheels moved
      for(int i = 0; i < 2; i++){
        double diff = wheelValues[i] - lastWheelValues[i];
        if(Math.abs(diff) < 0.001){
          diff = 0;
          wheelValues[i] = lastWheelValues[i];
        }
        distTraveled[i] = diff * encoderUnit;
      }
      
      
      //compute linear and angular velocity from how much wheels moved
      double v=(distTraveled[0]+distTraveled[1])/2.0;
      double w=(distTraveled[0]-distTraveled[1])/distBetweenWheels;
      
      double dt = 1;
      robotPose[2] += (w*dt);
      
      double vx = v*Math.cos(robotPose[2]);
      double vy = v*Math.sin(robotPose[2]);
      
      robotPose[0] += (vx*dt);
      robotPose[1] += (vy*dt);
      System.out.println(robotPose[0]+" "+robotPose[1]+" "+robotPose[2]);
      
      // deciding if there are obstacles on the left and, or right
      boolean right_obstacle =
        psValues[0] > 80.0 ||
        psValues[1] > 80.0;
      boolean left_obstacle =
        psValues[7] > 80.0 ||
        psValues[6] > 80.0;
      System.out.println(right_obstacle+" "+left_obstacle);


      //calculating direction in which to go
      retDir = Math.atan(robotPose[1]/robotPose[0]);
      if(robotPose[0]>0)
        retDir = retDir+Math.PI;
      
      retDir=(retDir+(2*Math.PI))%(2*Math.PI);
      robotPose[2] =(robotPose[2]+(2*Math.PI))%(2*Math.PI);
      System.out.println(retDir+" "+robotPose[2]);

      
      //  deciding if to stop or turn away from obstacle or go forward
      if(robotPose[0] > -returnPointEpsilon &&
        robotPose[0] < returnPointEpsilon &&
        robotPose[1] > -returnPointEpsilon &&
        robotPose[1] < returnPointEpsilon){
          leftMotor.setVelocity(0);
          rightMotor.setVelocity(0);
          break;
      }else if(left_obstacle||right_obstacle || closeToObsticle>0){//avoidingObsticles
        if(closeToObsticle<1)
          closeToObsticle = 30;
        leftMotor.setVelocity(0.5*MAX_SPEED);
        rightMotor.setVelocity(0.5*MAX_SPEED);
        if (left_obstacle) {
          // turn right
          leftMotor.setVelocity(0.5*MAX_SPEED);
          rightMotor.setVelocity(-0.5*MAX_SPEED);
        }
        else if (right_obstacle) {
           // turn left
           leftMotor.setVelocity(-0.5*MAX_SPEED);
           rightMotor.setVelocity(0.5*MAX_SPEED);         
         }
         closeToObsticle--;
      }else if(!((retDir+retDirEpsilon>robotPose[2])&&
                 (retDir-retDirEpsilon<robotPose[2]))){//turning to target
          if((retDir-robotPose[2])>0){
            //turn right
            leftMotor.setVelocity(0.1*MAX_SPEED);
            rightMotor.setVelocity(-0.1*MAX_SPEED);
          }else{
            //turn left
            leftMotor.setVelocity(-0.1*MAX_SPEED);
            rightMotor.setVelocity(0.1*MAX_SPEED);
          } 
       }else{
         //go straight
         leftMotor.setVelocity(0.5*MAX_SPEED);
         rightMotor.setVelocity(0.5*MAX_SPEED);
      }
      

      //update wheel values
      for(int i = 0; i < 2; i++){
        lastWheelValues[i] = wheelValues[i];
      }
    };
    
    leftMotor.setVelocity(0);
    rightMotor.setVelocity(0);

    while (robot.step(TIME_STEP) != -1) {
    };
    // Enter here exit cleanup code.
  }
  
  //read from file
  static int[][] readData()
  {
    int[][] matrix = new int[64][64];
    int x=0, y=0;

    try {
    BufferedReader in = new BufferedReader(new FileReader("D:\\Stud\\4-1\\robotai\\project\\RobotProject\\map_to_read.txt"));    //reading files in specified directory
  
      String line;
      while ((line = in.readLine()) != null)    //file reading
      {
        String[] values = line.split(",");
        for (String str : values)
        {
          int str_int = Integer.parseInt(str);
          matrix[x][y]=str_int;
          //System.out.print(str_int+" ");
          y=y+1;
        }
       //System.out.println();
      x=x+1;
      y=0;
      }
      in.close();
  
    }catch( IOException ioException ) {}
    return matrix;
  }
  
   // A utility function to print the constructed distance array
  static void printSolution(int dist[], int n)
  {
    //System.out.println("Vertex   Distance from Source");
    for (int i = 0; i < V; i++)
      System.out.println(i + " tt " + dist[i]);
  }
  
  static int minDistance(int dist[], Boolean sptSet[])
  {
    // Initialize min value
    int min = Integer.MAX_VALUE, min_index = -1;

    for (int v = 0; v < V; v++)
      if (sptSet[v] == false && dist[v] <= min) {
        min = dist[v];
        min_index = v;
      }

    return min_index;
  }
  
  static void getTargetRoad(int[] minRoads,int vrt, List<Integer> minRoad){
    if(minRoads[vrt]==-2){
      minRoad.add(vrt);
      return;
    }
    getTargetRoad(minRoads,minRoads[vrt],minRoad);
    minRoad.add(vrt);
  }
  
  // Function that implements Dijkstra's single source shortest path
  // algorithm for a graph represented using adjacency matrix
  // representation
  static ArrayList<Integer> dijkstra(int graph[][], int src,int goal, int[] minRoads)
  {
    int dist[] = new int[V]; // The output array. dist[i] will hold
    // the shortest distance from src to i

    // sptSet[i] will true if vertex i is included in shortest
    // path tree or shortest distance from src to i is finalized
    Boolean sptSet[] = new Boolean[V];

    // Initialize all distances as INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++) {
      dist[i] = Integer.MAX_VALUE;
      sptSet[i] = false;
    }

    // Distance of source vertex from itself is always 0
    dist[src] = 0;

    // Find shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
      // Pick the minimum distance vertex from the set of vertices
      // not yet processed. u is always equal to src in first
      // iteration.
      int u = minDistance(dist, sptSet);

      // Mark the picked vertex as processed
      sptSet[u] = true;

      // Update dist value of the adjacent vertices of the
      // picked vertex.
      for (int v = 0; v < V; v++)

        // Update dist[v] only if is not in sptSet, there is an
        // edge from u to v, and total weight of path from src to
        // v through u is smaller than current value of dist[v]
        if (!sptSet[v] && graph[u][v] != 0 && 
          dist[u] != Integer.MAX_VALUE && dist[u] + graph[u][v] < dist[v]){
          dist[v] = dist[u] + graph[u][v];
          minRoads[v]=u;
          }
    }

    // print the constructed distance array
    //printSolution(dist, V);
    
    //get da road
    ArrayList<Integer> minRoad = new ArrayList<Integer>();
    getTargetRoad(minRoads,goal,minRoad);
    for(int i = 0; i <minRoad.size();i++){
    //  System.out.print(minRoad.get(i)+" ");
    }
    //System.out.println();
    return minRoad;
  }
}
