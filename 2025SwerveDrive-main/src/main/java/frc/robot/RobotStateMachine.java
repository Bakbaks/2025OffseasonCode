package frc.robot;


import java.util.Map;


import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Units;    
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.ArmCommand.ArmSetPositionCommand;
import frc.robot.commands.ElevatorCommand.ElevatorSetPositionCommand;
import frc.robot.commands.GroundIntakeCommands.SpinGroundIntakeCommand;
import frc.robot.commands.GroundIntakeCommands.SwingGroundIntakeCommand;
import frc.robot.commands.IntakeCommand.IntakeSpinCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.SpinGroundIntakeSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.SwingGroundIntakeSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ArmConstant;
import frc.robot.Constants.GroundIntakeConstants;
import static java.util.Map.entry;


public class RobotStateMachine {
    private RobotState current = RobotState.START_CONFIG;


    public void setState(RobotState newState){
        current = newState;
    }


    public RobotState getState(){
        return current;
    }


    public Command build(ElevatorSubsystem elevator,
                         ArmSubsystem arm,
                         SwingGroundIntakeSubsystem swing,
                         SpinGroundIntakeSubsystem spin,
                         IntakeSubsystem intake){

       
            Map<RobotState, Command> states = Map.ofEntries(
                entry(RobotState.START_CONFIG,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters)))
                    ),
                   
                    Commands.deadline(
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("START_CONFIG")),


                entry(RobotState.INTAKE_DOWN,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing LOWERED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                            //.alongWith(Commands.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARGH")),
                        new IntakeSpinCommand(intake, 0)
                    ),


                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing LOWERED: " + GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        , new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0)
                            //.alongWith(Commands.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARGH")),
                        , new IntakeSpinCommand(intake, 0)
                    ),
                   
                   


                    Commands.deadline(
                        Commands.waitSeconds(1),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing LOWERED: " + GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        , new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new IntakeSpinCommand(intake, 0),
                        new SpinGroundIntakeCommand(spin, 0.7)
                            //.alongWith(Commands.print("GRAHRHRHHGHHGHGHGHGHGGHGHGHGHGH"))
                    )
                ).withName("INTAKE_DOWN")),


                entry(RobotState.HANDOFF, // your state 2
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing LOWERED: " + GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        , new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new IntakeSpinCommand(intake, 0),
                        new SpinGroundIntakeCommand(spin, 0.7) // start spin
                    ),
                   
                   


                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SpinGroundIntakeCommand(spin, 0.7),
                        new IntakeSpinCommand(intake, 0)
                    ),
                   


                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.ELEVATOR_HANDOFF_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.ELEVATOR_HANDOFF_DELTA.in(Meters))),
                        , new SpinGroundIntakeCommand(spin, 0.7),
                        new IntakeSpinCommand(intake, -0.6)
                    ),


                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.ELEVATOR_HANDOFF_DELTA)
                            //.alongWith(Commands.print("Elevator BASE: " + ElevatorConstants.ELEVATOR_HANDOFF_DELTA.in(Meters))),
                        , new SpinGroundIntakeCommand(spin, -0.7).withTimeout(1),
                        new IntakeSpinCommand(intake, -0.6).withTimeout(1)
                    ),


                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    )


                ).withName("HANDOFF")),
               
                entry(RobotState.L1,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LEVEL1_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_LEVEL1_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0.6),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("L1")),
               
                entry(RobotState.L2,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    ),
                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_2_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_2_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    )
                ).withName("L2")),


                entry(RobotState.L3,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_3_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_3_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    ),
                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_3_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_3_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    )
                ).withName("L3")),


                entry(RobotState.L4,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    ),
                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.6)
                    )
                ).withName("L4")),


                entry(RobotState.SCORE2,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_2_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_2_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(0.15),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.SCORE_STAGE_2_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_2_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_2_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),


                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.SCORE_STAGE_2_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_2_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_2_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        //new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
                        new IntakeSpinCommand(intake, 0.3).withTimeout(1.0)
                    )
               
                ).withName("SCORE2")),
               
                entry(RobotState.SCORE3,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_3_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_3_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(0.15),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.SCORE_STAGE_3_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_3_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_3_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),


                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.SCORE_STAGE_3_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_3_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_3_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        //new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
                        new IntakeSpinCommand(intake, 0.3).withTimeout(1.0)
                    )
               
                ).withName("SCOREL3")),


                entry(RobotState.SCORE4, 
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(0.15),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.3)
                    ),


                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.SCORE_STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        //new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
                        new IntakeSpinCommand(intake, 0.3).withTimeout(1.0)
                    )
               
                ).withName("SCOREL4")),




                entry(RobotState.SCOREL1,
                new SequentialCommandGroup(
                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LEVEL1_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_LEVEL1_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, -0.3).withTimeout(1.0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("SCOREL1")),

                entry(RobotState.SupaPinchHIGHGrab,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_HIGH_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_PINCH_HEIGHT_DELTA.in(Meters)))
                    ),
                   
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_HIGH_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_PINCH_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_PINCH_HIGH_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERGRAB_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.8)
                    )
                ).withName("SupaPinchHIGHGrab")),

                entry(RobotState.SupaPinchLOWGrab,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_LOW_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_PINCH_HEIGHT_DELTA.in(Meters)))
                    ),
                   
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_LOW_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.STAGE_PINCH_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_PINCH_LOW_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERGRAB_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.8)
                    )
                ).withName("SupaPinchLOWGrab")),
                
                entry(RobotState.SupaPinchTravel,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_MEDIUM_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters)))
                    ),
                   
                    Commands.deadline(
                        Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_MEDIUM_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.8)
                    )
                ).withName("SupaPinchTravel")),

                entry(RobotState.SupaPinchBarge,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_LINEUP_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters)))
                    ),
                   
                    Commands.deadline(
                        Commands.waitSeconds(0.2),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_LINEUP_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.8)
                    )
                ).withName("SupaPinchBarge")),

                entry(RobotState.SupaPinchScore,
                new SequentialCommandGroup(

                    Commands.deadline(
                        Commands.waitSeconds(0.325),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_SCORE_HEIGHT_DELTA),
                        new ArmSetPositionCommand(arm, ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees)),
                        new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, -0.8)
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))),
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters)))
                    ),

                    Commands.deadline(
                        //Commands.waitSeconds(0.25),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_PINCH_SCORE_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 1)
                    )
                ).withName("SupaPinchScore")),
                

                entry(RobotState.ClimbUp,
                new SequentialCommandGroup(

                    Commands.deadline(
                        Commands.waitSeconds(0.05),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.CLIMB_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("ClimbUp")),

                entry(RobotState.ClimbDown,
                new SequentialCommandGroup(

                    Commands.deadline(
                    
                        Commands.waitSeconds(0.05),
                        new ElevatorSetPositionCommand(elevator, ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            //.alongWith(Commands.print("Elevator default: " + ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        , new SwingGroundIntakeCommand(swing, GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Swing FEED: " + GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        , new ArmSetPositionCommand(arm, ArmConstant.CLIMB_ANGLE_VERTICAL.in(Degrees))
                            //.alongWith(Commands.print("Arm BASE: " + ArmConstant.ARM_SUPERSCORE_ANGLE_VERTICAL.in(Degrees))),
                        , new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    
                    )
                ).withName("ClimbDown"))
               
            );


            return new SelectCommand<RobotState>(states, ()->current).withName("SM" + current).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
           


       
    }
}



