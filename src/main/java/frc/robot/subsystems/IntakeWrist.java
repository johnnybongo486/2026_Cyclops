package frc.robot.subsystems;

import frc.lib.models.*;
import frc.robot.Constants;
import frc.robot.DeviceIds;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MatchLog;

public class IntakeWrist extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set Different Heights
	private double homePosition = Constants.Intake.IntakeWrist.StoreIntakePosition;
	private double maxUpTravelPosition = Constants.Intake.IntakeWrist.RunIntakePosition;

	public double upPositionLimit = maxUpTravelPosition;
	public double downPositionLimit = homePosition;
	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = -0.05;
	public double shooterAddValue;

	private final static double onTargetThreshold = 0.1;
		
	private TalonFX intakeWristKraken = new TalonFX(DeviceIds.Intake.WristMotorId, "canivore");
    private TalonFXConfiguration intakeWristFXConfig = new TalonFXConfiguration();

	public IntakeWrist() {
		// Clear Sticky Faults
		this.intakeWristKraken.clearStickyFaults();
		
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		intakeWristFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeWristFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
		intakeWristFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeWristFXConfig.CurrentLimits.StatorCurrentLimit = 50;

        /* PID Config */
        intakeWristFXConfig.Slot0.kP = 0.3;
        intakeWristFXConfig.Slot0.kI = 0.0;
        intakeWristFXConfig.Slot0.kD = 0.03;

        /* Open and Closed Loop Ramping */
        intakeWristFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
        intakeWristFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;

        intakeWristFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        intakeWristFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        //Config Acceleration and Velocity
        intakeWristFXConfig.MotionMagic.withMotionMagicAcceleration(600);
        intakeWristFXConfig.MotionMagic.withMotionMagicCruiseVelocity(300);

        // Config Motor
        intakeWristKraken.getConfigurator().apply(intakeWristFXConfig);
        intakeWristKraken.getConfigurator().setPosition(0.0);

		// Enable FOC
		targetPositionDutyCycle.withEnableFOC(true);
	}

	public void positionControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.intakeWristKraken.setControl(targetPositionDutyCycle);
	}

	public double getCurrentPosition() {
		return this.intakeWristKraken.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.intakeWristKraken.getSupplyCurrent().getValueAsDouble();
	}

	public boolean isHoldingPosition() {
		return this.isHoldingPosition;
	}

	public void setIsHoldingPosition(boolean isHoldingPosition) {
		this.isHoldingPosition = isHoldingPosition;
	}

	public double getTargetPosition() {
		return this.targetPosition;
	}

	public boolean setTargetPosition(double position) {
		if (!isValidPosition(position)) {
			return false;
		} else {
			if (position != this.targetPosition) {
				MatchLog.event("intakeWrist/targetPosition",
					String.format("%.3f -> %.3f rot", this.targetPosition, position));
			}
			this.targetPosition = position;
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		if (position != this.targetPosition) {
			MatchLog.event("intakeWrist/targetPosition",
				String.format("%.3f -> %.3f rot (forced)", this.targetPosition, position));
		}
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {		// && isintakeWristSafe(newTargetPosition) check for other subsystems
			this.targetPosition = newTargetPosition;
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= upPositionLimit && position >= downPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getHomePosition() {
		return this.homePosition;
	}

	public double getMaxUpTravelPosition() {
		return this.maxUpTravelPosition;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetintakeWristEncoder() {
        try {
			intakeWristKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("IntakeWrist.resetIntakeWristEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double joystickIntakeWrist(){
		double value = 0;
		value = -Robot.robotContainer.getOperatorLeftStickY();
		return value;
	}

	public double getPositionError() {
		double currentPosition = this.getCurrentPosition();
		double targetPosition = this.getTargetPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		return positionError;
	}

	public void manageMotion(double targetPosition) {
		double currentPosition = getCurrentPosition();
		if (currentPosition < targetPosition) {
				// set based on gravity
		}
		else {
				//set based on gravity
		}
	}

	public void zeroTarget() {
		targetPosition = 0;
	}

	public void updateDashboard() {
		SmartDashboard.putNumber("IntakeWrist Position", this.getCurrentPosition());
		SmartDashboard.putNumber("IntakeWrist Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("IntakeWrist Position Error", this.getPositionError());
		SmartDashboard.putNumber("IntakeWrist Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("IntakeWrist Current", this.getCurrentDraw());
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.intakeWristKraken.getVelocity().getValueAsDouble();
		return currentVelocity;
	}

	@Override
	public boolean isInPosition(double targetPosition) {
		double currentPosition = this.getCurrentPosition();
		double positionError = Math.abs(currentPosition - targetPosition);
		if (positionError < onTargetThreshold) {
			return true;
		} else {
			return false;
		}
	}
}   
