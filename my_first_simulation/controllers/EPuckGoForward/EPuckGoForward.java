// File:          EPuckGoForward.java
// Date:
// Description:
// Author:
// Modifications:

// You may need to add other webots classes such as
//  import com.cyberbotics.webots.controller.DistanceSensor;
//  import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Camera;



// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class EPuckGoForward {

  public static void main(String[] args) {

    int TIME_STEP = 64;
    
    System.out.println("start");
    
    double MAX_SPEED = 6.28;
    // create the Robot instance.
    Robot robot = new Robot();
    
    // initialize devices
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

    
    // get a handler to the motors and set target position to infinity (speed control)
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
   
    // set up the motor speeds at 10% of the MAX_SPEED.
    leftMotor.setVelocity(0.1 * MAX_SPEED);
    rightMotor.setVelocity(0.1 * MAX_SPEED);
   
    //create position sensor instances
    var left_ps = robot.getPositionSensor("left wheel sensor");
    left_ps.enable(TIME_STEP);
    var right_ps = robot.getPositionSensor("right wheel sensor");
    right_ps.enable(TIME_STEP);
   
    double[]target = {0.625,0}; //------target here
    double targetEpsilon = 0.005;

    double[] wheelValues = {0,0};
    double[] lastWheelValues = {0,0};
    double[] distTraveled = {0,0};
   
    double wheelRadius = 0.0205;
    double distBetweenWheels = 0.052;
    double wheelCirum = 2*3.14*wheelRadius;
    double encoderUnit = wheelCirum/6.28;
   
    //robot pose
    double[] robotPose={0,0,0};//x,y,theta
      
    double targetDir = Math.tan(target[1]-robotPose[1]/target[0]-robotPose[0]);
    double targetDirEpsilon = 0.02;
    
    int closeToObsticle = 0;
    boolean done = false;
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(TIME_STEP) != -1) {
        
      // Read the position sensors:
      wheelValues[0] = left_ps.getValue();
      wheelValues[1] = right_ps.getValue();
      
      double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i < 8 ; i++){
        psValues[i] = ps[i].getValue();
        //System.out.print(psValues[i]+" ");
      }
      System.out.println();
      // Process sensor data here.
      for(int i = 0; i < 2; i++){
        double diff = wheelValues[i] - lastWheelValues[i];
        if(Math.abs(diff) < 0.001){
          diff = 0;
          wheelValues[i] = lastWheelValues[i];
        }
        distTraveled[i] = diff * encoderUnit;
      }
      
      // detect obstacles
      boolean right_obstacle =
        psValues[0] > 80.0 ||
        psValues[1] > 80.0 ||
        psValues[2] > 80.0;
      boolean left_obstacle =
        psValues[7] > 80.0 ||
        psValues[6] > 80.0 ||
        psValues[5] > 80.0;
      //System.out.println(right_obstacle+" "+left_obstacle);
      //compute linear and angular velocity
      double v=(distTraveled[0]+distTraveled[1])/2.0;
      double w=(distTraveled[0]-distTraveled[1])/distBetweenWheels;
      
      double dt = 1;
      robotPose[2] += (w*dt);
      robotPose[2] =robotPose[2]%(2*Math.PI);
      
      double vx = v*Math.cos(robotPose[2]);
      double vy = v*Math.sin(robotPose[2]);
      
      robotPose[0] += (vx*dt);
      robotPose[1] += (vy*dt);
      //System.out.println(robotPose[0]+" "+robotPose[1]+" "+robotPose[2]);
      
      targetDir = Math.atan((target[1]-robotPose[1])/(target[0]-robotPose[0]));
      //System.out.println(targetDir+" "+robotPose[2]);
      // Enter here functions to send actuator commands, like:
      //  motor.setPosition(10.0);
      if(left_obstacle||right_obstacle || closeToObsticle>0){//avoidingObsticles
        if(closeToObsticle<1)
          closeToObsticle = 30;
        leftMotor.setVelocity(0.5*MAX_SPEED);
        rightMotor.setVelocity(0.5*MAX_SPEED);
        int[] image = camera.getImage();
        int pixel = image[image.length/2];
        int r = Camera.pixelGetRed(pixel);
        int g = Camera.pixelGetGreen(pixel);
        int b = Camera.pixelGetBlue(pixel);
        System.out.println("red=" + r + " green=" + g + " blue=" + b);

        if(r>100&&(psValues[7] > 80.0 ||psValues[0] > 80.0)){
          // turn right
          leftMotor.setVelocity(0.5*MAX_SPEED);
          rightMotor.setVelocity(-0.5*MAX_SPEED);
        }else if(g >100&&(psValues[7] > 80.0 ||psValues[0] > 80.0)){
           // turn left
           leftMotor.setVelocity(-0.5*MAX_SPEED);
           rightMotor.setVelocity(0.5*MAX_SPEED);         
        }else if (left_obstacle) {
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
      }else if(!((targetDir+targetDirEpsilon>robotPose[2])&&(targetDir-targetDirEpsilon<robotPose[2]))){//turning to target
          if((targetDir-robotPose[2])>0){//turn right
            leftMotor.setVelocity(0.1*MAX_SPEED);
            rightMotor.setVelocity(-0.1*MAX_SPEED);
          }else{//turn left
            leftMotor.setVelocity(-0.1*MAX_SPEED);
            rightMotor.setVelocity(0.1*MAX_SPEED);
          } 
       }else if(true){//go straight
          leftMotor.setVelocity(0.5*MAX_SPEED);
          rightMotor.setVelocity(0.5*MAX_SPEED);
      }
      
      if(robotPose[0] > target[0] - targetEpsilon &&
        robotPose[0] < target[0] + targetEpsilon &&
        robotPose[1] > target[1] - targetEpsilon &&
        robotPose[1] < target[1] + targetEpsilon){
          leftMotor.setVelocity(0);
          rightMotor.setVelocity(0);
          done = true;
      }
      
      for(int i = 0; i < 2; i++){
        lastWheelValues[i] = wheelValues[i];
      }
      if (done){
        leftMotor.setVelocity(0);
        rightMotor.setVelocity(0);
      }
    };

    // Enter here exit cleanup code.
  }
}
