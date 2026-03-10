package frc.robot.commands.Swerve;

import frc.robot.generated.TunerConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

//import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopDrive extends Command {
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

	private CommandSwerveDrivetrain drivetrain;
	private SwerveRequest.FieldCentric drive;
	private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;

	/* Setting up bindings for necessary control of the swerve drive platform */
	//private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
	
	public TeleopDrive(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
		this.drivetrain = drivetrain;
		addRequirements(drivetrain);
		this.drive = drive;
		this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.05);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.05);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.05);
        MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
		MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

		drivetrain.setControl(
			drive.withVelocityX(translationVal * MaxSpeed) // Drive forward with negative Y (forward)
			.withVelocityY(strafeVal * MaxSpeed) // Drive left with negative X (left)
			.withRotationalRate(rotationVal * MaxAngularRate) // Drive counterclockwise with negative X (left)
		);
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}