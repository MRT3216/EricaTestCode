
package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	Joystick xBox;
	// java is so much simpler in this respect:
	Talon frontLeft,frontRight,backLeft,backRight;
	
    public void robotInit() {
    	// wpilibj uses zero-based indices for components this year, 
    	// but last year wpilibc use 1-based indices (including joysticks)
    	xBox       = new Joystick(0);
		frontLeft  = new Talon(2);
		backLeft   = new Talon(1);
		frontRight = new Talon(0);		
		backRight  = new Talon(3);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	// Erica has no auton thus far
    }

    /**
     * This function is called periodically during operator control
     */
    // variables used in teleop:
    double lYaxis, triggers, rXaxis;
    
    public void teleopPeriodic() {
    	// this took a lot of guess-and-check, since several 
    	// axes were messed up this year. don't trust the 
    	// joystick explorer, use the DS for testing these
    	lYaxis   = 0 - xBox.getRawAxis(4);
    	rXaxis   = 0 - xBox.getRawAxis(1);	
    	// the triggers last year were read as a single value,
    	// but this year they are seperate
		triggers = xBox.getRawAxis(2) - xBox.getRawAxis(3);
		
		// code goes here (processing of non-drivetrain stuff)
		
		
		// call the function to set the motor speeds
		MecanumCartesian(triggers, lYaxis, rXaxis);
    }
    
    // the motors or gearboxes or something spin slower backwards,
    // so we multiply them by this constant to normalize them
    double kmap = 1.2;  // multiply backwards-spinning motors by this because they spin slower
    
    void MecanumCartesian(double triggers2, double lYaxis2, double rXaxis2){
    	// set up variables
		double forward, right, clockwise, temp, max;
		double front_left, front_right, rear_left, rear_right;
		forward = -lYaxis2; right = -triggers2; clockwise = rXaxis2;
		
		// scaling (used for practice)
		clockwise *= 0.50;
		//right     *= 0.50;
		//forward   *= 0.50;
		
		// dead zone
		if ((right     >-0.1)&&(right     <0.1)) right     = 0;
		if ((clockwise >-0.1)&&(clockwise <0.1)) clockwise = 0;
		if ((forward   >-0.1)&&(forward   <0.1)) forward   = 0;
		
		// reverse kinematics (thanks Eric!)
		front_left = forward + clockwise + right;
		front_right = forward - clockwise - right;
		rear_left = forward + clockwise - right;
		rear_right = forward - clockwise + right;
		
		// map backwards spinning motors
		if (front_left < 0) front_left *= kmap;
		if (front_right < 0) front_right *= kmap;
		if (rear_left < 0) rear_left *= kmap;
		if (rear_right < 0) rear_right *= kmap;
		
		// normalize so all are from 0 to 1
		max=Math.abs(front_right);
		if (max < Math.abs(front_left)) max = Math.abs(front_left);
		if (max < Math.abs(rear_left)) max = Math.abs(rear_left);
		if (max < Math.abs(rear_right)) max = Math.abs(rear_right);
		if (max>1) {
			front_left/=max; 
			front_right/=max; 
			rear_left/=max; 
			rear_right/=max;
		}
		 
		// finally, set the motors
		frontLeft.set(front_left);
		frontRight.set(front_right);
		backLeft.set(rear_left);
		backRight.set(rear_right);
	}
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	// no testing period
    }
    
}
