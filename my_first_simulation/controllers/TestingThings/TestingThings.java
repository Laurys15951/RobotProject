// File:          controllerForTestingThings.java
// Date:
// Description:
// Author:
// Modifications:

import com.cyberbotics.webots.controller.Robot;
import com.cyberbotics.webots.controller.Motor;


// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class TestingThings {

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


    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);

    // set up the motor speeds at 10% of the MAX_SPEED.
    leftMotor.setVelocity(0.5 * MAX_SPEED);
    rightMotor.setVelocity(0.5 * MAX_SPEED);

    //create position sensor instances
    var left_ps = robot.getPositionSensor("left wheel sensor");
    left_ps.enable(TIME_STEP);
    var right_ps = robot.getPositionSensor("right wheel sensor");
    right_ps.enable(TIME_STEP);

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

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot.step(timeStep) != -1) {

      // Read the position sensors:
      wheelValues[0] = left_ps.getValue();
      wheelValues[1] = right_ps.getValue();

      // Process sensor data here.
      for(int i = 0; i < 2; i++){
        double diff = wheelValues[i] - lastWheelValues[i];
        if(Math.abs(diff) < 0.001){
          diff = 0;
          wheelValues[i] = lastWheelValues[i];
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

      if(distTraveled[0]==0.125&&distTraveled[0]==0.125){
        //done = true;
      }


      // Process sensor data here.


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
