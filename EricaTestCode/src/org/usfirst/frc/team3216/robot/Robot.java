
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
	Talon frontLeft,frontRight,backLeft,backRight;
	
    public void robotInit() {
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
    	
    }

    /**
     * This function is called periodically during operator control
     */
    double lYaxis, triggers, rXaxis;
    public void teleopPeriodic() {
    	lYaxis   = xBox.getRawAxis(0);
		triggers = xBox.getRawAxis(1);
		rXaxis   = xBox.getRawAxis(2);		
		MecanumCartesian(triggers, lYaxis, rXaxis);
    }
    
    void MecanumCartesian(double triggers2, double lYaxis2, double rXaxis2){

		double forward, right, clockwise, temp, max;
		double front_left, front_right, rear_left, rear_right;
		forward = -lYaxis2; right = -triggers2; clockwise = rXaxis2;
		
		clockwise *= 0.50;
		/*right     *= 0.50;
		forward   *= 0.50;*/
		
		if ((right     >-0.1)&&(right     <0.1)) right     = 0;
		if ((clockwise >-0.1)&&(clockwise <0.1)) clockwise = 0;
		if ((forward   >-0.1)&&(forward   <0.1)) forward   = 0;
		
		front_left = forward + clockwise + right;
		front_right = forward - clockwise - right;
		rear_left = forward + clockwise - right;
		rear_right = forward - clockwise + right;
		
		max  = Math.abs(front_left);
		temp = Math.abs(front_right);
		if (temp>max) max = temp;
		temp = Math.abs(rear_left);
		if (temp>max) max = temp;
		temp = Math.abs(rear_right);
		if (temp>max) max = temp;

		if (max>1) {
			front_left/=max; 
			front_right/=max; 
			rear_left/=max; 
			rear_right/=max;
		}
		 
		frontLeft.set(front_left);
		frontRight.set(front_right);
		backLeft.set(rear_left);
		backRight.set(rear_right);
	}
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
