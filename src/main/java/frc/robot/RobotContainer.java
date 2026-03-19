package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoLowerIntake;
import frc.robot.commands.Auto.AutoRaiseIntake;
import frc.robot.commands.CANdle.SetColorFlow;
import frc.robot.commands.Hood.JoystickHood;
import frc.robot.commands.Hood.SetHoodPosition;
import frc.robot.commands.Serializer.StopUptake;
import frc.robot.commands.Serializer.RunAgitator;
import frc.robot.commands.Serializer.RunUptake;
import frc.robot.commands.Serializer.SlowAgitator;
import frc.robot.commands.Serializer.StopAgitator;
import frc.robot.commands.Shooter.AimToShootPoseOnly;
import frc.robot.commands.Shooter.ContinuousSetShooterAndHood;
import frc.robot.commands.Shooter.JoystickShooter;
import frc.robot.commands.Shooter.SetShooterVelocity;
import frc.robot.commands.Swerve.TeleopDrive;
import frc.robot.dashboard.AutoAimDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.Intake.JoystickIntakeWrist;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.SetIntakeWristPosition;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterLimelight;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.PoseEst;
import frc.robot.subsystems.PoseLimelight;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Setting up bindings for necessary control of the swerve drive platform */
  // No velocity-level deadband here — TeleopDrive already applies a 5% stick deadband via
  // MathUtil.applyDeadband before scaling to m/s. A second deadband here compounded to ~15%
  // stick deflection before the robot moved.
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /**
   * PROTOTYPE MODE — used by AutoAimPose and AimToShootPoseOnly to rotate the whole
   * robot toward the aiming target instead of moving the (non-existent) turret motor.
   *
   * Heading controller starting gains (tune on the practice field):
   *   kP = 5.0  (rad/s per rad error — increase if rotation is sluggish)
   *   kI = 0.0
   *   kD = 0.05 (increase slightly if it overshoots)
   *
   * Note: the physical turret used kP=0.8, kI=0, kD=0.01 in duty_cycle/encoder-rotation
   * units, which do not map numerically to the heading controller's rad/s per rad units.
   *
   * To restore the physical turret: comment out the drivetrain requirement in AutoAimPose
   * and AimToShootPoseOnly, uncomment turretKraken.setControl() in Turret.positionControl(),
   * and remove this field.
   */
  public static final SwerveRequest.FieldCentricFacingAngle facingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // The robot's subsystems and commands are defined here...
  public static Shooter shooter = new Shooter(); 
  public static Hood hood = new Hood();
  public static Uptake uptake = new Uptake();
  public static Agitator agitator = new Agitator();
  public static IntakeWrist intakeWrist = new IntakeWrist();
  public static Intake intake = new Intake();
  public static ShooterLimelight shooterLimelight = new ShooterLimelight();
  public static PoseEst poseEst = new PoseEst();
  public static PoseLimelight poseLimelight = new PoseLimelight();
  public static CANdleSubsystem caNdleSubsystem = new CANdleSubsystem();

  /* Path follower */
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Establish Controllers
  public final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

        private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // PROTOTYPE MODE: configure heading controller for turret-to-drivetrain redirect.
    // Tune kP first (raise until rotation tracks quickly without oscillating), then add kD.
    facingAngle.HeadingController.setPID(5.0, 0.0, 0.05);
    facingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    registerNamedCommands();

    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");
    // autoChooser.addOption("MidPassRight", new PathPlannerAuto("MidPassRight"));
    // autoChooser.addOption("MidPassLeft", new PathPlannerAuto("MidPassLeft"));
    // autoChooser.addOption("DoubleShotRight", new PathPlannerAuto("DoubleShotRight"));
    // autoChooser.addOption("DoubleShotRightSteal", new PathPlannerAuto("DoubleShotRightSteal"));
    // autoChooser.addOption("DoubleShotLeft", new PathPlannerAuto("DoubleShotLeft"));
    // autoChooser.addOption("DoubleShotLeftDepot", new PathPlannerAuto("DoubleShotLeftDepot"));
    // autoChooser.addOption("Depot", new PathPlannerAuto("Depot"));
    autoChooser.addOption("CyclopsRight", new PathPlannerAuto("CyclopsRight"));
    autoChooser.addOption("CyclopsTest", new PathPlannerAuto("CyclopsTest"));

    autoTab.add("Mode", autoChooser);
    
    // Set Default Commands
    uptake.setDefaultCommand(new StopUptake());
    agitator.setDefaultCommand(new StopAgitator());
    intake.setDefaultCommand(new StopIntake());
    
    drivetrain.setDefaultCommand(
      new TeleopDrive(drivetrain,
      drive,
      () -> -driverController.getRawAxis(translationAxis),
      () -> -driverController.getRawAxis(strafeAxis),
      () -> -driverController.getRawAxis(rotationAxis)  
      )
    );
    
    // Joysticks Used for Tuning
    hood.setDefaultCommand(new JoystickHood());
    shooter.setDefaultCommand(new JoystickShooter());
    //intakeWrist.setDefaultCommand(new JoystickIntakeWrist());

    // Configure the trigger bindings
    configureBindings();

    AutoAimDashboard.AddDashboard();
      
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.

    // Shoot
    driverController.rightBumper().whileTrue(new RunAgitator().alongWith(new RunUptake().alongWith(new SetColorFlow())));
    driverController.rightBumper().onFalse(new StopUptake().alongWith(new SlowAgitator()));//.alongWith(new SetIntakeWristPosition(5.15)));

    // drop intake
    driverController.b().onTrue(new SetIntakeWristPosition(5.15));

    // pick up intake
    driverController.b().multiPress(2,1).onTrue(new SetIntakeWristPosition(0));

    // aim using pose only
    driverController.leftTrigger().whileTrue(new AimToShootPoseOnly());
    driverController.leftTrigger().onFalse(new ContinuousSetShooterAndHood());

    //intake
    driverController.leftBumper().onTrue(new RunIntake().alongWith(new SlowAgitator()));
    driverController.leftBumper().multiPress(2, 1).onTrue(new StopIntake().alongWith(new StopAgitator()));

    // Operator Emergency Fix Intake Belt
    operatorController.x().whileTrue(new ReverseIntake());
    operatorController.x().onFalse(new RunIntake());

  final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );
  }

  /* Sets Joystick Deadband */
    public static double stickDeadband(double value, double deadband, double center) {
        return (value < (center + deadband) && value > (center - deadband)) ? center : value;
    }

    /* Passes Along Joystick Values for Elevator and Wrist */
    public double getOperatorLeftStickY() {
       return stickDeadband(this.operatorController.getRawAxis(1), 0.05, 0.0);
    }

    public double getOperatorLeftStickX() {
        return stickDeadband(this.operatorController.getRawAxis(0), 0.05, 0.0);
    }

    public double getOperatorRightStickY() {
        return stickDeadband(this.operatorController.getRawAxis(5), 0.05, 0.0);
    }

    public void doubleRumble() {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
    }

    public void stopRumble() {
        driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
    }

    // facing away from our alliance station wall (0 deg).
    //drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void registerNamedCommands() {
        /* Command registration for PathPlanner */     
        NamedCommands.registerCommand("LowerIntake", new AutoLowerIntake().withTimeout(0.1));
        NamedCommands.registerCommand("RaiseIntake", new AutoRaiseIntake());
        NamedCommands.registerCommand("AutoSetShooterAndHood", new ContinuousSetShooterAndHood());
        NamedCommands.registerCommand("AimToShoot", new AimToShootPoseOnly());
        NamedCommands.registerCommand("AimToShoot8", new AimToShootPoseOnly().withTimeout(1.25));
        NamedCommands.registerCommand("AimToShoot20", new AimToShootPoseOnly().withTimeout(3));
        NamedCommands.registerCommand("AimToShoot30", new AimToShootPoseOnly().withTimeout(4));
        NamedCommands.registerCommand("AimToShoot60", new AimToShootPoseOnly().withTimeout(6));
        NamedCommands.registerCommand("ShootCommand", new RunAgitator().alongWith(new RunUptake().alongWith(new RunIntake().withTimeout(2.5))));
        NamedCommands.registerCommand("RunIntake", new RunIntake().alongWith(new SlowAgitator()));
        NamedCommands.registerCommand("StopShoot", new StopUptake());
    }
}
