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
import com.cyberbotics.webots.controller.Accelerometer;



// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class EPuckGoForward {

  public static void main(String[] args) {

    int TIME_STEP = 64;
    
    System.out.println("start");
    
    double MAX_SPEED = 6.28;
    // create the Robot instance.
    Robot robot = new Robot();

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
   
   double[]target = {0.625,0.25};
   double targetEpsilon = 0.005;

   double[] psValues = {0,0};
   double[] lastPsValues = {0,0};
   double[] distTraveled = {0,0};
   
   double wheelRadius = 0.0205;
   double distBetweenWheels = 0.052;
   double wheelCirum = 2*3.14*wheelRadius;
   double encoderUnit = wheelCirum/6.28;
   
   //robot pose
   double[] robotPose={0,0,0};//x,y,theta
      
    double targetDir = Math.tan(target[1]-robotPose[1]/target[0]-robotPose[0]);
    double targetDirEpsilon = 0.02;
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(TIME_STEP) != -1) {
      
      // Read the position sensors:
      psValues[0] = left_ps.getValue();
      psValues[1] = right_ps.getValue();
         
      // Process sensor data here.
      for(int i = 0; i < 2; i++){
        double diff = psValues[i] - lastPsValues[i];
        if(Math.abs(diff) < 0.001){
          diff = 0;
          psValues[i] = lastPsValues[i];
        }
        distTraveled[i] = diff * encoderUnit;
      }
      
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
      if(false){//avoidingObsticles
      
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
      }
      
      for(int i = 0; i < 2; i++){
        lastPsValues[i] = psValues[i];
      }
      
    };

    // Enter here exit cleanup code.
  }
}
