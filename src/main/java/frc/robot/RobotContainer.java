package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoLowerIntake;
import frc.robot.commands.Hood.JoystickHood;
import frc.robot.commands.Hood.SetHoodPosition;
import frc.robot.commands.Serializer.StopUptake;
import frc.robot.commands.Serializer.RunAgitator;
import frc.robot.commands.Serializer.RunUptake;
import frc.robot.commands.Serializer.SlowAgitator;
import frc.robot.commands.Serializer.StopAgitator;
import frc.robot.commands.Shooter.AimToShootPoseOnly;
import frc.robot.commands.Shooter.ContinuousSetShooter;
import frc.robot.commands.Shooter.JoystickShooter;
import frc.robot.commands.Shooter.SetShooterVelocity;
import frc.robot.commands.Shooter.ShooterAdderCommand;
import frc.robot.commands.Swerve.TeleopDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.Intake.JoystickIntakeWrist;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.RunIntakeAuto;
import frc.robot.commands.Intake.RunIntakeSlow;
import frc.robot.commands.Intake.SetIntakeWristPosition;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.Agitator;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

  public static double standardDeviation = 0.7;

  // The robot's subsystems and commands are defined here...
  public static Shooter shooter = new Shooter(); 
  public static Hood hood = new Hood();
  public static Uptake uptake = new Uptake();
  public static Agitator agitator = new Agitator();
  public static IntakeWrist intakeWrist = new IntakeWrist();
  public static Intake intake = new Intake();
  public static PoseEst poseEst = new PoseEst();
  public static PoseLimelight poseLimelight = new PoseLimelight();

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
    /*
    autoChooser.addOption("DoubleShotRightSteal", new PathPlannerAuto("DoubleShotRightSteal"));
    autoChooser.addOption("DoubleShotRightLose", new PathPlannerAuto("DoubleShotRightLose"));
    autoChooser.addOption("DoubleShotRightSafe", new PathPlannerAuto("DoubleShotRightSafe"));
    autoChooser.addOption("DoubleShotLeftSteal", new PathPlannerAuto("DoubleShotLeftSteal"));
    autoChooser.addOption("DoubleShotLeftSafe", new PathPlannerAuto("DoubleShotLeftSafe"));
    autoChooser.addOption("DoubleShotLeftLose", new PathPlannerAuto("DoubleShotLeftLose"));
    autoChooser.addOption("PostUVMShortDoubleShotRightSafe", new PathPlannerAuto("PostUVMShortDoubleShotRightSafe"));
    autoChooser.addOption("PostUVMDoubleShotRightSafe", new PathPlannerAuto("PostUVMDoubleShotRightSafe"));
    autoChooser.addOption("PostUVMShortDoubleShotRightSteal", new PathPlannerAuto("PostUVMShortDoubleShotRightSteal"));


    autoChooser.addOption("ShortDoubleShotRightSteal", new PathPlannerAuto("ShortDoubleShotRightSteal"));
    autoChooser.addOption("ShortDoubleShotRightLose", new PathPlannerAuto("ShortDoubleShotRightLose"));
    autoChooser.addOption("ShortDoubleShotRightSafe", new PathPlannerAuto("ShortDoubleShotRightSafe"));
    autoChooser.addOption("ShortDoubleShotLeftSteal", new PathPlannerAuto("ShortDoubleShotLeftSteal"));
    autoChooser.addOption("ShortDoubleShotLeftSafe", new PathPlannerAuto("ShortDoubleShotLeftSafe"));
    autoChooser.addOption("ShortDoubleShotLeftLose", new PathPlannerAuto("ShortDoubleShotLeftLose"));
    */

    autoChooser.addOption("DCMPDoubleShotRightSafe", new PathPlannerAuto("DCMPDoubleShotRightSafe"));
    autoChooser.addOption("DCMPShortDoubleShotRightSafe", new PathPlannerAuto("DCMPShortDoubleShotRightSafe"));
    autoChooser.addOption("DCMPShortDoubleShotRightSteal", new PathPlannerAuto("DCMPShortDoubleShotRightSteal"));


    autoTab.add("Mode", autoChooser);
    
    // Set Default Commands
    uptake.setDefaultCommand(new StopUptake());
    agitator.setDefaultCommand(new StopAgitator());
    intake.setDefaultCommand(new RunIntakeAuto());

    // for pit testing, comment out the hood command below and uncomment the intake command below
    hood.setDefaultCommand(new ContinuousSetShooter());
    // intake.setDefaultCommand(new StopIntake());

    
    drivetrain.setDefaultCommand(
      new TeleopDrive(drivetrain,
      drive,
      () -> -driverController.getRawAxis(translationAxis),
      () -> -driverController.getRawAxis(strafeAxis),
      () -> -driverController.getRawAxis(rotationAxis)  
      )
    );
    
    // Joysticks Used for Tuning
    // hood.setDefaultCommand(new JoystickHood());
    // shooter.setDefaultCommand(new JoystickShooter());
    // intakeWrist.setDefaultCommand(new JoystickIntakeWrist());

    // Configure the trigger bindings
    configureBindings();      
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
    driverController.rightBumper().whileTrue((new ShooterAdderCommand(Constants.Shooter.ShooterSpeed.ShooterAdder).withTimeout(0.25).andThen(new ShooterAdderCommand(0))).alongWith(new RunAgitator().alongWith(new RunIntakeSlow()).alongWith(new RunUptake()).alongWith(new WaitCommand(0.5).andThen(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.squeeze)))));//2
    driverController.rightBumper().onFalse(new StopUptake().alongWith(new RunIntakeAuto()).alongWith(new SlowAgitator().alongWith(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.RunIntakePosition))));  //6.25

    // drop intake
    driverController.b().onTrue(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.RunIntakePosition));

    // pick up intake
    driverController.b().multiPress(2,1).onTrue(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.StoreIntakePosition));

    // aim using pose only
    driverController.leftTrigger().whileTrue(new AimToShootPoseOnly());
    driverController.leftTrigger().onFalse(new ContinuousSetShooter());

    //intake
    driverController.leftBumper().onTrue(new RunIntakeAuto().alongWith(new SlowAgitator()));
    driverController.leftBumper().multiPress(2, 1).onTrue(new StopIntake().alongWith(new StopAgitator()));

    // driver and operator override shooter adder
    operatorController.a().and(driverController.rightBumper()).whileTrue(new RunAgitator().alongWith(new RunIntakeSlow()).alongWith(new RunUptake()).alongWith(new WaitCommand(0.5).andThen(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.squeeze))));

    // Operator Emergency Fix Intake Belt
    operatorController.x().whileTrue(new ReverseIntake());
    operatorController.x().onFalse(new RunIntakeAuto());

    operatorController.y().whileTrue(new StopIntake());
    operatorController.y().onFalse(new RunIntakeAuto());

    // for testing
    // operatorController.a().onTrue(new SetHoodPosition(2));
    // operatorController.b().onTrue(new SetHoodPosition(8));

    // Operator Fixed Position Shooting
    // bumpers on Hub, intake against ladders, side shoot from trenches
    operatorController.rightTrigger().and(operatorController.a()).whileTrue(new SetHoodPosition(Constants.Shooter.FixedShootHood.bumpers).alongWith(new SetShooterVelocity(Constants.Shooter.FixedShootSpeed.bumpers)));
    operatorController.rightTrigger().and(operatorController.x()).whileTrue(new SetHoodPosition(Constants.Shooter.FixedShootHood.ladder).alongWith(new SetShooterVelocity(Constants.Shooter.FixedShootSpeed.ladder)));
    operatorController.rightTrigger().and(operatorController.b()).whileTrue(new SetHoodPosition(Constants.Shooter.FixedShootHood.side).alongWith(new SetShooterVelocity(Constants.Shooter.FixedShootSpeed.side)));
    
    // reset after fixed shooting; or emergency hood to down position
    operatorController.rightTrigger().and(operatorController.a()).onFalse(new SetHoodPosition(Constants.Shooter.Hood.StoreHoodPosition));
    operatorController.rightTrigger().and(operatorController.x()).onFalse(new SetHoodPosition(Constants.Shooter.Hood.StoreHoodPosition));
    operatorController.rightTrigger().and(operatorController.b()).onFalse(new SetHoodPosition(Constants.Shooter.Hood.StoreHoodPosition));

    
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
        NamedCommands.registerCommand("LowerIntake", new AutoLowerIntake().withTimeout(0.01));
        NamedCommands.registerCommand("AutoSetShooterAndHood", new ContinuousSetShooter());
        NamedCommands.registerCommand("AimToShoot", new AimToShootPoseOnly());
        NamedCommands.registerCommand("ShootCommand", (new ShooterAdderCommand(Constants.Shooter.ShooterSpeed.ShooterAdder).withTimeout(0.25).andThen(new ShooterAdderCommand(0))).alongWith(new RunAgitator().alongWith(new RunIntakeSlow()).alongWith(new RunUptake()).alongWith(new WaitCommand(0.5).andThen(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.squeeze)))));
        NamedCommands.registerCommand("RunIntake", new RunIntake());
        NamedCommands.registerCommand("StopShoot", new StopUptake().alongWith(new RunIntake()).alongWith(new SlowAgitator().alongWith(new SetIntakeWristPosition(Constants.Intake.IntakeWrist.RunIntakePosition))));
    }
}
