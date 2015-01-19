package org.usfirst.frc.team3216.robot;

import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

/**********
 * Controls overview:
 * left joystick - forward and backward controls forward speed
 * right joystick - left and right controls rotation
 * triggers - difference between triggers controls strafe
 * bumpers - left bumper lowers claw
 *         - right bumper raises claw
 * buttons - b opens claw
 *         - x closes claw
 *         - a
 *         - y
 * direction pad -
 * 
 * @author jacksonservheen
 *
 */
public class Robot extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
	Timer clawTimer;
	PowerDistributionPanel pdp;
	Joystick xBox;
	Compressor pcm;
	Relay winch;
	// java is so much simpler in this respect:
	Talon frontLeft,frontRight,backLeft,backRight;
	DoubleSolenoid claw;
	
    public void robotInit() {
    	// wpilibj uses zero-based indices for components this year, 
    	// but last year wpilibc use 1-based indices (including joysticks)
    	clawTimer = new Timer();
    	xBox       = new Joystick(0);
    	pdp = new PowerDistributionPanel();
		frontLeft  = new Talon(2);
		backLeft   = new Talon(1);
		frontRight = new Talon(0);		
		backRight  = new Talon(3);
		pcm = new Compressor(0);
		claw = new DoubleSolenoid(0,1);  //switch these numbers if it goes backwards
		winch = new Relay(0);
		
		pcm.setClosedLoopControl(true);
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
		// call the function to set the motor speeds
		MecanumCartesian(triggers, lYaxis, rXaxis);
		
		
		// manage the claw - buttons x and b
		int clawdir = 0;
		boolean open = xBox.getRawButton(2), close = xBox.getRawButton(3);
		if (open) clawdir = 1;
		if (close) clawdir = -1;
		manageClaw(clawdir);
		
		// manage the winch - bumpers
		int winchdir = 0;
		boolean up = xBox.getRawButton(6), down = xBox.getRawButton(5);
		if (up) winchdir = -1;
		if (down) winchdir = 1;
		manageWinch(winchdir);
    }
    
    void manageClaw(int direction) {
    	// pass 1 to open
    	//     -1 to close
    	//      0 to do nothing
    	
    	// this will stop the solenoid 30 msec after no buttons are pressed
    	if (direction == 0 && clawTimer.get() > 0.3) {  //TODO: fix this logic -- i think its broken
    		claw.set(DoubleSolenoid.Value.kOff);
    		clawTimer.stop();
    		clawTimer.reset();
    	}
    	else if (direction == 1) {
    		claw.set(DoubleSolenoid.Value.kForward);
    		clawTimer.start();
    	}
    	else if (direction == -1) {
    		claw.set(DoubleSolenoid.Value.kReverse);
    		clawTimer.start();
    	}
    }
    
    void manageWinch(int direction) {
    	// pass 1 to open
    	//     -1 to close
    	//      0 to do nothing
    	
    	// fairly simple logic: move based on buttons.
    	if (direction == 0) {  
    		winch.set(Relay.Value.kOff);
    	}
    	else if (direction == 1) {
    		winch.set(Relay.Value.kForward);
    	}
    	else if (direction == -1) {
    		winch.set(Relay.Value.kReverse);
    	}
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
