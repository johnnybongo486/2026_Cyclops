package frc.robot.subsystems;

import frc.lib.LimelightHelpers;
import frc.lib.models.*;
import frc.robot.Constants;
import frc.robot.DeviceIds;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase implements IPositionControlledSubsystem {

	private boolean isHoldingPosition = false;

    // Set left and right limits
	public double rightPositionLimit = Constants.Shooter.Turret.MaximumTurretPosition;
	public double leftPositionLimit = Constants.Shooter.Turret.MinimumTurretPosition;

	private double targetPosition = 0;
    private MotionMagicDutyCycle targetPositionDutyCycle = new MotionMagicDutyCycle(0);
	private double feedForward = 0.0;
	public double shooterAddValue;
	public boolean onTarget = true;

	private final static double onTargetThreshold = 0.1;
		
	private TalonFX turretKraken = new TalonFX(DeviceIds.Shooter.TurretMotorId, "canivore");

    private TalonFXConfiguration turretFXConfig = new TalonFXConfiguration();

	public Turret() {
		// Clear Sticky Faults
		this.turretKraken.clearStickyFaults();
		
        /** Shooter Motor Configuration */
        /* Motor Inverts and Neutral Mode */
		turretFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        turretFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Current Limiting */
		turretFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turretFXConfig.CurrentLimits.StatorCurrentLimit = 35;

        /* PID Config */
        turretFXConfig.Slot0.kP = 0.8;  // 0.2
        turretFXConfig.Slot0.kI = 0;
        turretFXConfig.Slot0.kD = 0.01; // 0.01

        /* Open and Closed Loop Ramping */
        turretFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        turretFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;

        turretFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;
        turretFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;

        //Config Acceleration and Velocity
        turretFXConfig.MotionMagic.withMotionMagicAcceleration(300);
        turretFXConfig.MotionMagic.withMotionMagicCruiseVelocity(300);

        // Config Motor
        turretKraken.getConfigurator().apply(turretFXConfig);
        turretKraken.getConfigurator().setPosition(0.0);

		// Enable FOC
		targetPositionDutyCycle.withEnableFOC(true);

		resetturretEncoder();
	}

	public void positionControl() {
		this.manageMotion(targetPosition);
        targetPositionDutyCycle.withPosition(targetPosition);
        targetPositionDutyCycle.withFeedForward(feedForward);
		this.turretKraken.setControl(targetPositionDutyCycle);
	}

	public void positionManualControl(double targetPosition) {
		this.targetPosition = targetPosition;
	}

	public double getCurrentPosition() {
		return this.turretKraken.getRotorPosition().getValueAsDouble();
	}

	public double getCurrentDraw() {
		return this.turretKraken.getSupplyCurrent().getValueAsDouble();
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
			setOnTarget(false);
			RobotContainer.caNdleSubsystem.setAnimate("Strobe Red");
			return false;
		} else {
			this.targetPosition = position;
			setOnTarget(true);
			RobotContainer.caNdleSubsystem.setAnimate("Rainbow");
			return true;
		}
	}

	public void forceSetTargetPosition(double position) {
		this.targetPosition = position;
	}

	public void incrementTargetPosition(double increment) {
		double currentTargetPosition = this.targetPosition;
		double newTargetPosition = currentTargetPosition + increment;

		// check to see if the turret can point where you want it to
		if (isValidPosition(newTargetPosition)) {
			this.targetPosition = newTargetPosition;
			RobotContainer.caNdleSubsystem.setAnimate("Rainbow");
			setOnTarget(true);
		} 

		else {
			RobotContainer.caNdleSubsystem.setAnimate("Strobe Red");
			setOnTarget(false);
		}
	}

	public boolean isValidPosition(double position) {
		boolean withinBounds = position <= rightPositionLimit && position >= leftPositionLimit;
		return withinBounds;
	}

    // communicate with commands
	public double getLeftLimit() {
		return this.leftPositionLimit;
	}

	public double getRightLimit() {
		return this.rightPositionLimit;
	}

	public double getFeedForward() {
		return this.feedForward;
	}

	public void resetturretEncoder() {
        try {
			turretKraken.getConfigurator().setPosition(0.0);
        }
        catch (Exception e) {
            DriverStation.reportError("Turret.resetTurretEncoders exception.  You're Screwed! : " + e.toString(), false);
        }
	}

	public double joystickTurret(){
		double value = 0;
		value = Robot.robotContainer.getOperatorLeftStickX();
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
		SmartDashboard.putNumber("Turret Position", this.getCurrentPosition());
		SmartDashboard.putNumber("Turret Target Position", this.getTargetPosition());
		SmartDashboard.putNumber("Turret Position Error", this.getPositionError());
		SmartDashboard.putNumber("Turret Velocity", this.getCurrentVelocity());
		SmartDashboard.putNumber("Turret Current", this.getCurrentDraw());
		SmartDashboard.putNumber("Chassis Angle", getYaw().getDegrees()%360);
		SmartDashboard.putNumber("Chassis xSpeed", getXSpeed());
		SmartDashboard.putNumber("Chassis ySpeed", getYSpeed());
		SmartDashboard.putNumber("Chassis rSpeed", getRSpeed());
		SmartDashboard.putNumber("TagID", LimelightHelpers.getFiducialID("limelight-shooter"));
	}

	@Override
	public double getCurrentVelocity() {
		double currentVelocity = this.turretKraken.getVelocity().getValueAsDouble();
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
	
	public boolean getOnTarget(){
		return onTarget;
	}

	public void setOnTarget(boolean onTarget) {
		this.onTarget = onTarget;
	}

	public Rotation2d getYaw() {
		return RobotContainer.drivetrain.getPigeon2().getRotation2d();			// this method polls the pigeon in drivetrain
	}					

	public double getXSpeed(){
		ChassisSpeeds speed = RobotContainer.drivetrain.getState().Speeds;
		Rotation2d angle = RobotContainer.drivetrain.getState().Pose.getRotation();
		double xSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(speed,angle).vxMetersPerSecond;
		return xSpeed;
	}

	public double getYSpeed(){
		ChassisSpeeds speed = RobotContainer.drivetrain.getState().Speeds;
		Rotation2d angle = RobotContainer.drivetrain.getState().Pose.getRotation();
		double ySpeed = ChassisSpeeds.fromRobotRelativeSpeeds(speed,angle).vyMetersPerSecond;
		return ySpeed;
	}

	public double getRSpeed(){
		double rSpeed = RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond;
		return rSpeed;
	}
}   

