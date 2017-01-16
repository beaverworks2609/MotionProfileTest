/**
 * This Java FRC robot application is meant to demonstrate an example using the Motion Profile control mode
 * in Talon SRX.  The CANTalon class gives us the ability to buffer up trajectory points and execute them
 * as the roboRIO streams them into the Talon SRX.
 * 
 * There are many valid ways to use this feature and this example does not sufficiently demonstrate every possible
 * method.  Motion Profile streaming can be as complex as the developer needs it to be for advanced applications,
 * or it can be used in a simple fashion for fire-and-forget actions that require precise timing.
 * 
 * This application is an IterativeRobot project to demonstrate a minimal implementation not requiring the command 
 * framework, however these code excerpts could be moved into a command-based project.
 * 
 * The project also includes instrumentation.java which simply has debug printfs, and a MotionProfile.java which is generated
 * in @link https://docs.google.com/spreadsheets/d/1PgT10EeQiR92LNXEOEe3VGn737P7WDP4t0CQxQgC8k0/edit#gid=1813770630&vpid=A1
 * 
 * Logitech Gamepad mapping, use left y axis to drive Talon normally.  
 * Press and hold top-left-shoulder-button5 to put Talon into motion profile control mode.
 * This will start sending Motion Profile to Talon while Talon is neutral. 
 * 
 * While holding top-left-shoulder-button5, tap top-right-shoulder-button6.
 * This will signal Talon to fire MP.  When MP is done, Talon will "hold" the last setpoint position
 * and wait for another button6 press to fire again.
 * 
 * Release button5 to allow OpenVoltage control with left y axis.
 */

package org.usfirst.frc.team2609.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {

	/** The Talon we want to motion profile. */
	CANTalon _right1talon = new CANTalon(1);
	CANTalon _rightSlavetalon = new CANTalon(2);
	CANTalon _left1talon = new CANTalon(3);
	CANTalon _leftSlavetalon = new CANTalon(4);
	
	

	/** some example logic on how one can manage an MP */
	MotionProfileExample _leftexample = new MotionProfileExample(_left1talon);
	MotionProfileExample _rightexample = new MotionProfileExample(_right1talon);
	
	/** joystick for testing */
	Joystick _joy= new Joystick(0);

	/** cache last buttons so we can detect press events.  In a command-based project you can leverage the on-press event
	 * but for this simple example, lets just do quick compares to prev-btn-states */
	boolean [] _btnsLast = {false,false,false,false,false,false,false,false,false,false};
	public void robotInit(){
		_rightSlavetalon.changeControlMode(TalonControlMode.Follower);
		_leftSlavetalon.changeControlMode(TalonControlMode.Follower);
		_rightSlavetalon.set(1);
		_leftSlavetalon.set(3);
		_right1talon.setInverted(false);
	}

	public Robot() { // could also use RobotInit()
		_right1talon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		_right1talon.reverseSensor(true); /* keep sensor and motor in phase */		
		_right1talon.configEncoderCodesPerRev(250);
		_left1talon.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		_left1talon.reverseSensor(true); /* keep sensor and motor in phase */
		_left1talon.configEncoderCodesPerRev(350);
	}
	/**  function is called periodically during operator control */
    public void teleopPeriodic() {
		/* get buttons */
		boolean [] btns= new boolean [_btnsLast.length];
		for(int i=1;i<_btnsLast.length;++i)
			btns[i] = _joy.getRawButton(i);

		/* get the left joystick axis on Logitech Gampead */
		double leftYjoystick = -1 * _joy.getY(); /* multiple by -1 so joystick forward is positive */

		/* call this periodically, and catch the output.  Only apply it if user wants to run MP. */
		_leftexample.control();
		_rightexample.control();
		
		if (btns[5] == false) { /* Check button 5 (top left shoulder on the logitech gamead). */
			/*
			 * If it's not being pressed, just do a simple drive.  This
			 * could be a RobotDrive class or custom drivetrain logic.
			 * The point is we want the switch in and out of MP Control mode.*/
		
			/* button5 is off so straight drive */
			_left1talon.changeControlMode(TalonControlMode.Voltage);
			_left1talon.set(12.0 * leftYjoystick);
			_right1talon.changeControlMode(TalonControlMode.Voltage);
			_right1talon.set(12.0 * leftYjoystick);

			_leftexample.reset();
			_rightexample.reset();
		} else {
			/* Button5 is held down so switch to motion profile control mode => This is done in MotionProfileControl.
			 * When we transition from no-press to press,
			 * pass a "true" once to MotionProfileControl.
			 */
			_left1talon.changeControlMode(TalonControlMode.MotionProfile);
			_right1talon.changeControlMode(TalonControlMode.MotionProfile);
			
			CANTalon.SetValueMotionProfile setOutputL = _leftexample.getSetValue();
			CANTalon.SetValueMotionProfile setOutputR = _rightexample.getSetValue();
					
			_left1talon.set(setOutputL.value);
			_right1talon.set(setOutputR.value);

			/* if btn is pressed and was not pressed last time,
			 * In other words we just detected the on-press event.
			 * This will signal the robot to start a MP */
			if( (btns[6] == true) && (_btnsLast[6] == false) ) {
				/* user just tapped button 6 */
				_leftexample.startMotionProfile();
				_rightexample.startMotionProfile();
			}
		}

		/* save buttons states for on-press detection */
		for(int i=1;i<10;++i)
			_btnsLast[i] = btns[i];

	}
	/**  function is called periodically during disable */
	public void disabledPeriodic() {
		/* it's generally a good idea to put motor controllers back
		 * into a known state when robot is disabled.  That way when you
		 * enable the robot doesn't just continue doing what it was doing before.
		 * BUT if that's what the application/testing requires than modify this accordingly */
		_left1talon.changeControlMode(TalonControlMode.PercentVbus);
		_left1talon.set( 0 );
		_right1talon.changeControlMode(TalonControlMode.PercentVbus);
		_right1talon.set( 0 );
		/* clear our buffer and put everything into a known state */
		_leftexample.reset();
		_rightexample.reset();
	}
}
