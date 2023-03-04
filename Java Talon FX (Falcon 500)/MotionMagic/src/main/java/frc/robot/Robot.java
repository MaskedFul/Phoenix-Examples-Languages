/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon Software Reference Manual.
 * 
 * Controls:
 * Button 1(Button A): When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2(Button B): When pushed, the selected feedback sensor gets zero'd
 * POV 180(Dpad Down): When pushed, will decrement the smoothing of the motion magic down to 0
 * POV 0(Dpad Up): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon FX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Maigic: Servo Talon FX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {

	XboxController xbox;

	DutyCycleEncoder angleEncoder;

	/* Setpoints */
	double m_targetMin = -727000;
	double m_targetMax = 18600;

	/* Hardware */
	WPI_TalonFX leftExtMotor = new WPI_TalonFX(2);
	WPI_TalonFX rightExtMotor = new WPI_TalonFX(13);
	WPI_TalonFX angMotor = new WPI_TalonFX(15); // Rename "rio" to match the CANivore device name if using a CANivore
	Joystick _joy = new Joystick(0);
	DutyCycleEncoder extEncoder = new DutyCycleEncoder(9);


	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public void simulationInit() {
		PhysicsSim.getInstance().addTalonFX(angMotor, 0.5, 5100);
	}
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public void robotInit() {
		/* Factory default hardware to prevent unexpected behavior */
		leftExtMotor.setInverted(true);
		rightExtMotor.setInverted(false);

		leftExtMotor.setNeutralMode(NeutralMode.Brake);
		rightExtMotor.setNeutralMode(NeutralMode.Brake);
		angMotor.setNeutralMode(NeutralMode.Brake);

		angleEncoder = new DutyCycleEncoder(0);

		angMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		angMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		angMotor.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		angMotor.setSensorPhase(false);
		angMotor.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		angMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		angMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		angMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
		angMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
		angMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
		angMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		angMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		angMotor.config_kF(Constants.kSlotIdx, SmartDashboard.getNumber("Feedforward Gain", 0.047), Constants.kTimeoutMs);
		angMotor.config_kP(Constants.kSlotIdx, SmartDashboard.getNumber("P Gain", 0), Constants.kTimeoutMs);
		angMotor.config_kI(Constants.kSlotIdx, SmartDashboard.getNumber("I Gain", 0), Constants.kTimeoutMs);
		angMotor.config_kD(Constants.kSlotIdx, SmartDashboard.getNumber("D Gain", 0), Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		angMotor.configMotionCruiseVelocity(21400, Constants.kTimeoutMs);
		angMotor.configMotionAcceleration(21400, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		//_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		angMotor.configFeedbackNotContinuous(true, Constants.kTimeoutMs);

		angMotor.config_IntegralZone(Constants.kSlotIdx, 3);
		
	}

	@Override
	public void teleopInit() {
		angMotor.set(TalonFXControlMode.PercentOutput, 0);
		angMotor.setSelectedSensorPosition(0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
 
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY(); /* left-side Y for Xbox360Gamepad */
		double rghtYstick = -1.0 * _joy.getRawAxis(5); /* right-side Y for Xbox360Gamepad */
		if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
		if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */
		
		/* Get current Talon FX motor output */
		double motorOutput = angMotor.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(angMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		_sb.append("\t Position:");
		_sb.append(angMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("Lift Position", angMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("Angle Encoder", angleEncoder.getAbsolutePosition());
		SmartDashboard.putNumber("Extension", extEncoder.getDistance());

		/* Arbirrary Feed Forward */
		//double horizontalHoldOutput = 0;
		//double arbfeedFwdTerm = getFeedForward(horizontalHoldOutput);

		/* Slide Extension */
			if (_joy.getRawButton(10)) {
	  
				extendArmUsingPowerNoLimit(rghtYstick / 1);
				resetExtensionEncoder();
		
			} else {
		
				extendArmUsingPower(rghtYstick / 1);
		
			}


		/**
		 * Perform Motion Magic when Button 1 is held, else run Percent Output, which can
		 * be used to confirm hardware setup.
		 */
		if (_joy.getRawButton(1)) {
			/* Motion Magic */

			/* 2048 ticks/rev * 10 Rotations in either direction */
			double targetPos = m_targetMin;
			angMotor.set(TalonFXControlMode.MotionMagic, targetPos);
			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(angMotor.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);

		} else if (_joy.getRawButton(4)) {
		
			double targetPos = m_targetMax;
			angMotor.set(TalonFXControlMode.MotionMagic, targetPos);
			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(angMotor.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
			
		} else {
			/* Percent Output */

			angMotor.set(TalonFXControlMode.PercentOutput, leftYstick);
		}

		if (_joy.getRawButton(2)) {
			/* Zero sensor positions */
			//angMotor.setSelectedSensorPosition(0);
		}

		int pov = _joy.getPOV();
		if (_pov == pov) {
			/* no change */
		} else if (_pov == 180) { // D-Pad down
			/* Decrease smoothing */
			_smoothing--;
			if (_smoothing < 0)
				_smoothing = 0;
			angMotor.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		} else if (_pov == 0) { // D-Pad up
			/* Increase smoothing */
			_smoothing++;
			if (_smoothing > 8)
				_smoothing = 8;
			angMotor.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		_pov = pov; /* save the pov value for next time */

		/* Instrumentation */
		Instrum.Process(angMotor, _sb);
	}

	private double getFeedForward(double horizontalHoldOutput) {
		double m_CurrentAngle = angleEncoder.getAbsolutePosition();

		double theta = Math.toRadians(90 - m_CurrentAngle);

		double gravityCompensation = Math.cos(theta);

		double arb_feedForward = gravityCompensation * horizontalHoldOutput;

		return arb_feedForward;
	}

	public void extendArmUsingPower(double speed) {
     
		if (speed > 0 && getExtensionEncoderInches() > Constants.maxExtensionInchValue) {
	
		  leftExtMotor.set(ControlMode.PercentOutput, 0);
		  rightExtMotor.set(ControlMode.PercentOutput, 0);
	
		} else if(speed < 0 && getExtensionEncoderInches() < Constants.minExtensionInchValue) {
	
		  leftExtMotor.set(ControlMode.PercentOutput, 0);
		  rightExtMotor.set(ControlMode.PercentOutput, 0);
			
		} else {
	
		  leftExtMotor.set(ControlMode.PercentOutput, speed);
		  rightExtMotor.set(ControlMode.PercentOutput, speed);
	
		}
	
		leftExtMotor.set(ControlMode.PercentOutput, speed);
		rightExtMotor.set(ControlMode.PercentOutput, speed);
		
	  }
	
	  public void extendArmUsingPowerNoLimit(double speed) {
		 
		leftExtMotor.set(ControlMode.PercentOutput, speed);
		rightExtMotor.set(ControlMode.PercentOutput, speed);
	 
	  }

	  public double getExtensionEncoderInches(){
    
		double encoderInches = extEncoder.getDistance() * 1.5 * Math.PI;
	
		return encoderInches + Constants.extOffset;
	  }
	
	  public void setExtensionOffset() {
	
		extEncoder.setPositionOffset(extEncoder.getDistance());
	
	  }
	
	  public void resetExtensionEncoder() {
	
		extEncoder.reset();
	
	  }
	}
