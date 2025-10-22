package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstant;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AimAtTagCommand;
import frc.robot.commands.AimAtTagPIDCommand;
import frc.robot.commands.ArmCommand.ArmSetPositionCommand;
import frc.robot.commands.AutoCommands.ArmAutonCommands;
import frc.robot.commands.AutoCommands.AutonIntakeWithDetectionCommand;
import frc.robot.commands.AutoCommands.ElevatorAutonComomands;
import frc.robot.commands.ElevatorCommand.ElevatorSetPositionCommand;
import frc.robot.commands.GroundIntakeCommands.SpinGroundIntakeCommand;
import frc.robot.commands.GroundIntakeCommands.SwingGroundIntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeHoldPositionCommand;
import frc.robot.commands.IntakeCommand.IntakeSpinCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmState;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.SpinGroundIntakeSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.SwingGroundIntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem.CommandSwerveDrivetrain;
import frc.robot.subsystems.SwerveSubsystem.TunerConstants;

public class RobotContainer {

    private final RobotStateMachine sm = new RobotStateMachine();


    
    // swerve drive stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
                                                                                    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.025).withRotationalDeadband(MaxAngularRate * 0.025) // Add a 2.5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // end of swerve drive things

    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    public static CommandXboxController m_auxController = new CommandXboxController(OIConstants.kAuxControllerPort);
    public static XboxController m_Controller = new XboxController(OIConstants.kAuxControllerPort);

    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final SwingGroundIntakeSubsystem SwingGroundIntake = new SwingGroundIntakeSubsystem();
    private final SpinGroundIntakeSubsystem spinGroundIntake = new SpinGroundIntakeSubsystem();
    PathConstraints lims = new PathConstraints(
    3.0,                     // max m/s
    1.0,                     // max m/s^2
    Math.toRadians(540.0),   // max rad/s
    Math.toRadians(720.0)    // max rad/s^2
    );
    /* 
    private final AimAtTagCommand aimAtTagL = new AimAtTagCommand(Constants.VisionConstants.TAG_TO_GOAL_LEFT, drivetrain, vision, lims, 0.1);
    private final AimAtTagCommand aimAtTagR = new AimAtTagCommand(Constants.VisionConstants.TAG_TO_GOAL_RIGHT, drivetrain, vision, lims, 0.1);
      */  

    private final AimAtTagPIDCommand aimAtTagL =
        new AimAtTagPIDCommand(Constants.VisionConstants.TAG_TO_GOAL_LEFT, drivetrain, vision);

    private final AimAtTagPIDCommand aimAtTagR =
        new AimAtTagPIDCommand(Constants.VisionConstants.TAG_TO_GOAL_RIGHT, drivetrain, vision);


    private final Trigger auxY = m_auxController.y();
    private final Trigger auxA = m_auxController.a();
    private final Trigger auxB = m_auxController.b();
    private final Trigger auxX = m_auxController.x();
    private final Trigger auxRightBumper = m_auxController.rightBumper();
    private final Trigger auxRightTrigger = m_auxController.rightTrigger();
    private final Trigger auxLeftBumper = m_auxController.leftBumper();
    private final Trigger auxLeftTrigger = m_auxController.leftTrigger();
    private final Trigger auxPovUP = m_auxController.povUp();
    private final Trigger auxPovDOWN = m_auxController.povDown();
    private final Trigger auxPovLEFT = m_auxController.povLeft();
    private final Trigger auxPovRIGHT = m_auxController.povRight();
        
    private final Trigger driveY = m_driverController.y();
    private final Trigger driveA = m_driverController.a();
    private final Trigger driveB = m_driverController.b();
    private final Trigger driveX = m_driverController.x();
    private final Trigger driveRightBumper = m_driverController.rightBumper();
    private final Trigger driveRightTrigger = m_driverController.rightTrigger();
    private final Trigger driveLeftBumper = m_driverController.leftBumper();
    private final Trigger driveLeftTrigger = m_driverController.leftTrigger();
    private final Trigger drivePovDOWN = m_driverController.povDown();
    private final Trigger drivePovUP = m_driverController.povUp();
    private boolean sadMode = false;
    private final SendableChooser<Command> autoChooser;


    //Fake button shenanigans
    // Software gate for right trigger
    private final java.util.concurrent.atomic.AtomicBoolean rtGate = new java.util.concurrent.atomic.AtomicBoolean(true);

    private final Trigger driveRightTriggerArmed = driveRightTrigger.and(new Trigger(rtGate::get));

    private final java.util.concurrent.atomic.AtomicBoolean ltGate = new java.util.concurrent.atomic.AtomicBoolean(true);

    private final Trigger driveLeftTriggerArmed = driveLeftTrigger.and(new Trigger(ltGate::get));

    
    private final java.util.concurrent.atomic.AtomicBoolean auxLBGate = new java.util.concurrent.atomic.AtomicBoolean(true);
    private final java.util.concurrent.atomic.AtomicBoolean auxLTGate = new java.util.concurrent.atomic.AtomicBoolean(true);

    private final Trigger auxLeftBumperArmed = auxLeftBumper.and(new Trigger(auxLBGate::get));
    private final Trigger auxLeftTriggerArmed = auxLeftTrigger.and(new Trigger(auxLTGate::get));

    private static Command pulseGate(java.util.concurrent.atomic.AtomicBoolean gate){
        return Commands.sequence(
                Commands.runOnce(()-> gate.set(false)),
                new edu.wpi.first.wpilibj2.command.WaitCommand(0.02),
                Commands.runOnce(()-> gate.set(true))
        );
    }

    Command AutoAimR = new SequentialCommandGroup(
        new AimAtTagCommand(Constants.VisionConstants.TAG_TO_GOAL_RIGHT, drivetrain, vision, lims, 0.1).withTimeout(1.5),
        new InstantCommand(() -> drivetrain.resetOdometry(drivetrain.getPose()))
        );
   Command AutoAimL = new SequentialCommandGroup(
        new AimAtTagCommand(Constants.VisionConstants.TAG_TO_GOAL_LEFT, drivetrain, vision, lims, 0.1).withTimeout(2),
        new InstantCommand(() -> drivetrain.resetOdometry(drivetrain.getPose()))
        );

    Command NewL4 = new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spinGroundIntake, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spinGroundIntake, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    )
                );
    Command NewScoreL4 = new SequentialCommandGroup(
        Commands.deadline(
            Commands.waitSeconds(0.1),
            new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
            , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            , new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
            , new SpinGroundIntakeCommand(spinGroundIntake, 0),
            new IntakeSpinCommand(intake, -0.3)
        ),

        Commands.deadline(
            Commands.waitSeconds(0.15),
            new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA)
                //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
            , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_4_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
            , new SpinGroundIntakeCommand(spinGroundIntake, 0),
            //new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
            new IntakeSpinCommand(intake, -0.3).withTimeout(0.5)
        ),

        Commands.deadline(
            Commands.waitSeconds(0.2),
            new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
            , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
            , new SpinGroundIntakeCommand(spinGroundIntake, 0),
            //new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
            new IntakeSpinCommand(intake, 0.3).withTimeout(1.0)
        )


   
    );

    Command NewDefault = new SequentialCommandGroup(
        Commands.deadline(
            Commands.waitSeconds(0.1),
            new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters)))
        ),
       
        Commands.deadline(
            Commands.waitSeconds(0.25),
            new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
            , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
            , new SpinGroundIntakeCommand(spinGroundIntake, 0.2),
            new IntakeSpinCommand(intake, -0.3)
        )
    );

    Command NewStationLoad = new SequentialCommandGroup(
        Commands.deadline(
            Commands.waitSeconds(3),
            new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
            , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            , new ArmSetPositionCommand(arm, ArmConstant.ARM_PINCH_HIGH_ANGLE_VERTICAL.in(Degrees))
                //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
            , new SpinGroundIntakeCommand(spinGroundIntake, 0.6),
            new IntakeSpinCommand(intake, 0)
        ),
        
        Commands.deadline(
                Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing LOWERED: " + GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        , new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new IntakeSpinCommand(intake, 0),
                        new SpinGroundIntakeCommand(spinGroundIntake, 0.6) // start spin
                ),
                   
                
                   


        Commands.deadline(
                Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.ELEVATOR_HANDOFF_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.ELEVATOR_HANDOFF_DELTA.in(Meters))),
                        , new SpinGroundIntakeCommand(spinGroundIntake, 0.6),
                        new IntakeSpinCommand(intake, -0.5)
                ),


                Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.ELEVATOR_HANDOFF_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.ELEVATOR_HANDOFF_DELTA.in(Meters))),
                        , new SpinGroundIntakeCommand(spinGroundIntake, -0.5).withTimeout(0.2),
                        new IntakeSpinCommand(intake, -0.5).withTimeout(0.2)
                ),


                Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new ElevatorSetPositionCommand(elevatorSubsystem, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(SwingGroundIntake, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spinGroundIntake, 0),
                        new IntakeSpinCommand(intake, -0.5)
                )

    );

    Command AEI_Scoring_L4 = new SequentialCommandGroup(
        new ParallelCommandGroup(
                new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA), 
                new ArmAutonCommands(arm,ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                ).withTimeout(2),
        new InstantCommand(() -> intake.feedWest()).withTimeout(2)
        );
    Command AEI_Scoring_L3 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.8),
            new InstantCommand(() -> intake.feedWest()).withTimeout(2)
                                            ).withTimeout(4);
    Command AEI_Scoring_L2 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.7),
            new InstantCommand(() -> intake.feedWest()).withTimeout(2)
                                            ).withTimeout(4);
    Command AEI_Scoring_L1 = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.6),
            new InstantCommand(() -> intake.feedWest()).withTimeout(2)
                                            ).withTimeout(4);
    Command AEI_Source = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
                                    ).withTimeout(1.9),
            new InstantCommand(() -> intake.feedEast()).withTimeout(2)
                                            ).withTimeout(4);
        Command AEI_Zero = new SequentialCommandGroup(
            new ParallelCommandGroup(
                                     new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA), 
                                     new ArmAutonCommands(arm,ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                                     ).withTimeout(1.6),
            new InstantCommand(() -> intake.stop()).withTimeout(2)
                                       );

                                       
        Command AEI_Scoring_L4_OCR_FIX = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA), 
                        new ArmAutonCommands(arm,ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                        ).withTimeout(1.3),
                        Commands.startEnd(()->intake.feedWest(),() -> intake.stop(),intake).withTimeout(2),
                        AEI_Zero
                );
        // Command SourceLoading = new SequentialCommandGroup(
        //         new ParallelCommandGroup(
        //                 new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA), 
        //                 new ArmAutonCommands(arm,ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees))
        //                 ).withTimeout(2),
        //                 new FunctionalCommand(
        //                 ()->{
        //                 intake.feedEast();
        //                 },
        //                 interrupted -> intake.stop,
        //                 () -> intake.getCANrangeRight().getDistance().getValue().in(Centimeter) < 18   
        //                 ,
        //                 intake),
        //                 AEI_Zero
        //         );



    public RobotContainer() {
        NamedCommands.registerCommand("NewDefault", NewDefault);
        NamedCommands.registerCommand("AutoAlignR", AutoAimR);
        NamedCommands.registerCommand("AutoAlignL", AutoAimL);
        NamedCommands.registerCommand("NewL4", NewL4);
        NamedCommands.registerCommand("NewScoreL4", NewScoreL4);
        NamedCommands.registerCommand("NewStationLoad", NewStationLoad);

        NamedCommands.registerCommand("Elevator_L4_Happy", new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA).withTimeout(1));

        NamedCommands.registerCommand("Elevator_Zero", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_Source_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L1_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L2_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L3_Happy", new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA).withTimeout(1));
        NamedCommands.registerCommand("Elevator_L4_Happy", new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA).withTimeout(1));

        NamedCommands.registerCommand("Arm_Zero", new ArmAutonCommands(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_Source_Happy", new ArmAutonCommands(arm, ArmConstant.CORAL_STATION_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L4_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L3_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L2_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));
        NamedCommands.registerCommand("Arm_L1_Happy", new ArmAutonCommands(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees)).withTimeout(1));


        NamedCommands.registerCommand("AE_L1_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_1_ANGLE_VERTICAL.in(Degrees))).withTimeout(2));
        NamedCommands.registerCommand("AE_L2_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))).withTimeout(2));
        NamedCommands.registerCommand("AE_L3_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))).withTimeout(2));
        NamedCommands.registerCommand("AE_L4_Happy", new ParallelCommandGroup(new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA), new ArmAutonCommands(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))).andThen(new InstantCommand(() -> intake.feedWest()).withTimeout(3)).withTimeout(2));
        NamedCommands.registerCommand("AE_Source_Happy", new ParallelCommandGroup( new ArmAutonCommands(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)), new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_SOURCE_DELTA)).withTimeout(2));

        NamedCommands.registerCommand("AE_Zero", new ParallelCommandGroup( new ArmAutonCommands(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),new ElevatorAutonComomands(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)).withTimeout(2));






        NamedCommands.registerCommand("Intake_Scoring_West", new InstantCommand(() -> intake.feedWest()).withTimeout(300));

        NamedCommands.registerCommand("Intake_Source", 
        new SequentialCommandGroup(new AutonIntakeWithDetectionCommand(intake, intake.getCANrangeLeft(),intake.getCANrangeRight(), true), 
                                   new IntakeHoldPositionCommand(intake)));

        NamedCommands.registerCommand("AEI_Scoring_L1", AEI_Scoring_L1);
        NamedCommands.registerCommand("AEI_Scoring_L2", AEI_Scoring_L2);
        NamedCommands.registerCommand("AEI_Scoring_L3", AEI_Scoring_L3);
        NamedCommands.registerCommand("AEI_Scoring_L4", AEI_Scoring_L4);
        NamedCommands.registerCommand("AEI_Source", AEI_Source);
        NamedCommands.registerCommand("AEI_Zero", AEI_Zero);
        NamedCommands.registerCommand("AEI_Scoring_L4_OCR_FIX", AEI_Scoring_L4_OCR_FIX);
        

        autoChooser = AutoBuilder.buildAutoChooser("Taxi");
        SmartDashboard.putData("Auto Chooser", autoChooser);





      
        configureBindings();
    }

    private void configureBindings() {
        // begin of swerve drive bindings
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain
                        .applyRequest(() -> drive
                                .withVelocityX(
                                        -mathProfiles.exponentialDrive(m_driverController.getLeftY(), 3) * MaxSpeed)
                                .withVelocityY(
                                        -mathProfiles.exponentialDrive(m_driverController.getLeftX(), 3) * MaxSpeed)
                                .withRotationalRate(-mathProfiles.exponentialDrive(m_driverController.getRightX(), 2)
                                        * MaxAngularRate))
        // drivetrain.applyRequest(() ->
        // drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
        // negative Y (forward)
        // .withVelocityY(-joystick.getLeftX()* MaxSpeed) // Drive left with negative X
        // (left)
        // .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
        // counterclockwise with negative X (left)
        // )
        );

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(
                        new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x())
                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on POV down press
        drivePovDOWN.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        // end of swerve drive bindings

        /* 
        driveRightTriggerArmed.onTrue(
                Commands.runOnce(() -> {
                        RobotState cur = sm.getState();
                        RobotState target;

                        if (cur == RobotState.INTAKE_DOWN) {
                                target = RobotState.HANDOFF;
                                } else {
                                target = RobotState.INTAKE_DOWN;
                                }

                                System.out.println("------------------------------ ROBOT STATE : " + cur + " -> " + target);

                                sm.setState(target);
                                

                })
                .andThen(Commands.parallel(
                new edu.wpi.first.wpilibj2.command.ProxyCommand(
                () -> sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake)
                        .withInterruptBehavior(edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior.kCancelSelf)
                        .withName("SM:" + sm.getState())
                ),
                pulseGate(rtGate) // re-arm regardless of command duration
        )));
        
        */
        /*
        driveLeftTrigger.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.HANDOFF))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        driveRightTrigger.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.INTAKE_DOWN))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        ); 
        */
        /* 
        driveRightTrigger.onTrue(
            Commands.runOnce(() -> System.out.println("on True"))
 
        ); 

        driveRightTrigger.onFalse(
            Commands.runOnce(() -> System.out.println("on False"))
          
        ); 
        */
        
        /* 
        driveA.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.SCOREL1))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        driveB.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.SCORE2))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        driveX.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.SCORE3))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        driveY.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.SCORE4))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        
        */
        //reviewed and passed
        driveRightTrigger.onTrue(
                Commands.runOnce(() -> {
                        RobotState cur = sm.getState();
                        RobotState target;

                        if (cur == RobotState.INTAKE_DOWN) {
                        target = RobotState.HANDOFF;
                        } else {
                        target = RobotState.INTAKE_DOWN;
                        }

                        System.out.println("------------------------------ ROBOT STATE : " + cur + " to " + target);
                        
                        sm.setState(target);
                })
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
                //.andThen(pulseGate(rtGate)).alongWith(Commands.print("RESET BUTTON-----------" + driveRightTriggerArmed.getAsBoolean()))
        );
        driveRightTrigger.onFalse(
            Commands.runOnce(() -> System.out.println("on False"))
        ); 
    
        /* 
        driveRightTriggerArmed.onTrue(
                Commands.runOnce(() -> {
                        RobotState cur = sm.getState();
                        RobotState target;

                        if (cur == RobotState.INTAKE_DOWN) {
                        target = RobotState.HANDOFF;
                        } else {
                        target = RobotState.INTAKE_DOWN;
                        }

                        System.out.println("------------------------------ ROBOT STATE : " + cur + " to " + target);
                        
                        sm.setState(target);
                })
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
                .andThen(pulseGate(rtGate))
        );
        */
        

        //
        driveLeftTrigger.onTrue(
        Commands.runOnce(() -> {
                RobotState cur = sm.getState();
                RobotState target;

                if (cur == RobotState.L1) {
                target = RobotState.SCOREL1;
                } else if (cur == RobotState.L2) {
                target = RobotState.SCORE2;
                } else if (cur == RobotState.L3) {
                target = RobotState.SCORE3;
                } else if (cur == RobotState.L4) {
                target = RobotState.SCORE4;
                } else if (cur == RobotState.SupaPinchBarge) {
                target = RobotState.SupaPinchScore;
                } else if (cur == RobotState.SCOREL1
                        || cur == RobotState.SCORE2
                        || cur == RobotState.SCORE3
                        || cur == RobotState.SCORE4
                        || cur == RobotState.SupaPinchScore) {
                target = cur; // already scoring or releasing; keep it
                } else {
                target = cur; // default
                }

                sm.setState(target);
        })   
        .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        

        driveRightBumper.whileTrue(aimAtTagR);
        driveLeftBumper.whileTrue(aimAtTagL);


        

        //AUX COMMANDS

        
        auxRightTrigger.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.START_CONFIG))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        

        auxA.onTrue(
                Commands.runOnce(() -> sm.setState(RobotState.L1))
                        .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        auxB.onTrue(
                Commands.runOnce(() -> sm.setState(RobotState.L2))
                        .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        auxX.onTrue(
                Commands.runOnce(() -> sm.setState(RobotState.L3))
                        .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );
        auxY.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.L4))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        /* 
        //default elevator and arm manual control
        m_auxController.start().whileTrue(new youPary(elevatorSubsystem));
        elevatorSubsystem.setDefaultCommand(new RunCommand(() -> {
            double rightXAxis = m_auxController.getRightY();
            elevatorSubsystem.manualControl(-rightXAxis*0.25+0.035);
        }, elevatorSubsystem)
        .alongWith(Commands.print("Elevator Manual Controlling: " + m_auxController.getRightX())));

        m_auxController.back().whileTrue(new youParyArm(arm));
        arm.setDefaultCommand(new RunCommand(() -> {
            double leftYAxis = m_auxController.getLeftX();
            arm.manualControl(-leftYAxis*0.2);
        }, arm)
        .alongWith(Commands.print("Arm Manual Controlling: "+m_auxController.getLeftY())));
        */
       

        auxLeftBumper.onTrue(
        Commands.runOnce(() -> {
                RobotState cur = sm.getState();
                RobotState target;
                
                if (cur == RobotState.SupaPinchHIGHGrab) {
                    target = RobotState.SupaPinchTravel;
                } else if(cur == RobotState.SupaPinchTravel) {
                    target = RobotState.SupaPinchBarge;
                }else{
                    target = RobotState.SupaPinchHIGHGrab;
                }

                System.out.println("------------------------------ ROBOT STATE : " + cur + " to " + target);
                

                sm.setState(target);
        })
        .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        auxLeftBumper.onFalse(
            Commands.runOnce(() -> System.out.println("on False"))
        ); 

        auxLeftTrigger.onTrue(
        Commands.runOnce(() -> {
                RobotState cur = sm.getState();
                RobotState target;

                if (cur == RobotState.SupaPinchLOWGrab) {
                    target = RobotState.SupaPinchTravel;
                } else if(cur == RobotState.SupaPinchTravel) {
                    target = RobotState.SupaPinchBarge;
                }else{
                    target = RobotState.SupaPinchLOWGrab;
                }

                

                sm.setState(target);
        })
        .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        

        
        //Climber Bindings

        auxPovUP.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.ClimbUp))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        auxPovUP.onTrue(new RunCommand(() -> {
            climb.expand();
        }, climb))
        .onFalse(new RunCommand(() -> {
            climb.stop();
        }, climb));

        auxPovDOWN.onTrue(
        Commands.runOnce(() -> sm.setState(RobotState.ClimbDown))
                .andThen(sm.build(elevatorSubsystem, arm, SwingGroundIntake, spinGroundIntake, intake))
        );

        auxPovDOWN.onTrue(new RunCommand(() -> {
            climb.retract();
        }, climb))
        .onFalse(new RunCommand(() -> {
            climb.stop();
        }, climb));

        //auxPovLEFT.onTrue(new RunCommand(() -> {intake.feedEast(0.1);}, intake)).onFalse(new RunCommand(() -> {intake.stop();}, intake));
        //auxPovRIGHT.onTrue(new RunCommand(() -> {intake.feedWest(0.1);}, intake)).onFalse(new RunCommand(() -> {intake.stop();}, intake));


       
        



    }

    public Command getAutonomousCommand() {
//        return new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA);
        return autoChooser.getSelected();
        // try {
        //         PathPlannerPath path = PathPlannerPath.fromPathFile("blueUpPreloadPath");
        //         return AutoBuilder.followPath(path);
        // } catch (FileVersionException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        // } catch (IOException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        // } catch (ParseException e) {
        //         // TODO Auto-generated catch block
        //         e.printStackTrace();
        // }
        // return Commands.none();


    }

    

    private void handleIntakeByArmState(ArmState state, double speed) {
    switch (state) {
        case LEVEL1:
        case LEVEL2:
        case SAD_LEVEL3:
        case SAD_LEVEL4:
            intake.feedEast(speed);
            break;
        case SAD_LEVEL1:
        case SAD_LEVEL2:
        case LEVEL3:
        case LEVEL4:
            intake.feedWest(speed);
            break;
        default:
            intake.stop();
            break;
    }

}
}
