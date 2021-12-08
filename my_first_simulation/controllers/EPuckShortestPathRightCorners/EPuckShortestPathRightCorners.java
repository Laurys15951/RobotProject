// File:          EPuckShortestPathRightCorners.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
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

    // get the motor devices
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    
    // set the target position of the motors
    leftMotor.setVelocity(0.5 * MAX_SPEED);
    rightMotor.setVelocity(0.5 * MAX_SPEED);

    var left_ps = robot.getPositionSensor("left wheel sensor");
    left_ps.enable(TIME_STEP);
    var right_ps = robot.getPositionSensor("right wheel sensor");
    right_ps.enable(TIME_STEP);

    double[] wheelValues = {0,0};
    double[] lastWheelValues = {0,0};

    //square forward 
    double forward = 6.24;
    //leftMotor.setPosition(6.24);
    //rightMotor.setPosition(6.24);
    double turn = 2.4;
    //leftMotor.setPosition(forward);
    //rightMotor.setPosition(forward);
    
    int start =17;
    int finish = 49;
    
    //path algorithm
    //Map graph
    int graph[][]=readData();
    int[] minRoads =new int[64];
    for(int i = 0; i< 64; i++){
      minRoads[i] = -1;
    }
    minRoads[start] = -2;
    ArrayList<Integer> road = dijkstra(graph, start, finish,minRoads);
    
    
    int botDir = 0;
    int step = 0;
    boolean turning = false;
    
    
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(TIME_STEP) != -1) {
      //System.out.println(left_ps.getValue()+" "+right_ps.getValue());
      // Read the sensors:
      // Enter here functions to read sensor data, like:
      wheelValues[0] = left_ps.getValue();
      wheelValues[1] = right_ps.getValue();

      // Process sensor data here.
      if(lastWheelValues[0]==wheelValues[0]&&lastWheelValues[1]==wheelValues[1] && turning){
            leftMotor.setPosition(wheelValues[0]+forward);
            rightMotor.setPosition(wheelValues[1]+forward);
            turning = false;
      } else if(lastWheelValues[0]==wheelValues[0]&&lastWheelValues[1]==wheelValues[1]){
            if(step+1 == road.size())
              break;
            int stepDif = road.get(step+1)-road.get(step);
            //System.out.println(stepDif+" "+botDir);
            if(stepDif==1){
              if(botDir==0){
                leftMotor.setPosition(wheelValues[0]+forward);
                rightMotor.setPosition(wheelValues[1]+forward);
              }else if(botDir==1){
                leftMotor.setPosition(wheelValues[0]-turn);
                rightMotor.setPosition(wheelValues[1]+turn);
                turning = true;
              }else if(botDir==3){
                leftMotor.setPosition(wheelValues[0]+turn);
                rightMotor.setPosition(wheelValues[1]-turn);
                turning = true;
              }
              botDir = 0;
            }else if(stepDif==8){
              if(botDir==1){
                leftMotor.setPosition(wheelValues[0]+forward);
                rightMotor.setPosition(wheelValues[1]+forward);
              }else if(botDir==2){
                leftMotor.setPosition(wheelValues[0]-turn);
                rightMotor.setPosition(wheelValues[1]+turn);
                turning = true;
              }else if(botDir==0){
                leftMotor.setPosition(wheelValues[0]+turn);
                rightMotor.setPosition(wheelValues[1]-turn);
                turning = true;
               }
               botDir = 1;
            }else if(stepDif==-1){
              if(botDir==2){
                leftMotor.setPosition(wheelValues[0]+forward);
                rightMotor.setPosition(wheelValues[1]+forward);
              }else if(botDir==3){
                leftMotor.setPosition(wheelValues[0]-turn);
                rightMotor.setPosition(wheelValues[1]+turn);
                turning = true;
              }else if(botDir==1){
                leftMotor.setPosition(wheelValues[0]+turn);
                rightMotor.setPosition(wheelValues[1]-turn);
                turning = true;
              }              
              botDir = 2;
            }else if(stepDif==-8){
              if(botDir==3){
                leftMotor.setPosition(wheelValues[0]+forward);
                rightMotor.setPosition(wheelValues[1]+forward);
              }else if(botDir==0){
                leftMotor.setPosition(wheelValues[0]-turn);
                rightMotor.setPosition(wheelValues[1]+turn);
                turning = true;
              }else if(botDir==2){
                leftMotor.setPosition(wheelValues[0]+turn);
                rightMotor.setPosition(wheelValues[1]-turn);
                turning = true;
              }
              botDir = 3;
            }
            step++;
      }


      // Enter here functions to send actuator commands, like:
      //  motor.setPosition(10.0);
      
      for(int i = 0; i < 2; i++){
        lastWheelValues[i] = wheelValues[i];
      }
    };

    // Enter here exit cleanup code.
  }
  
  //read from file
  static int[][] readData()
  {
    int[][] matrix = new int[64][64];
    int x=0, y=0;

    try
    {
    BufferedReader in = new BufferedReader(new FileReader("D:\\Stud\\4-1\\robotai\\map_to_read.txt"));    //reading files in specified directory
  
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
    System.out.println("Vertex   Distance from Source");
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
