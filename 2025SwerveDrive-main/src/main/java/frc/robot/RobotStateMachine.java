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


        
            Map<RobotState, Command> states = Map.of(
                RobotState.START_CONFIG,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters)))
                    ),
                    
                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("START_CONFIG"),

                RobotState.INTAKE_DOWN,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Swing LOWERED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                            //.alongWith(Commands.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARGH")),
                        new IntakeSpinCommand(intake, 0)
                    ),

                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Swing LOWERED: " + Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0)
                            .alongWith(Commands.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAARGH")),
                        new IntakeSpinCommand(intake, 0)
                    ),
                    
                    

                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Swing LOWERED: " + Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.ELEVATOR_BASE_DELTA.in(Meters))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new IntakeSpinCommand(intake, 0),
                        new SpinGroundIntakeCommand(spin, 0.3)
                            .alongWith(Commands.print("GRAHRHRHHGHHGHGHGHGHGGHGHGHGHGH"))
                    )
                ).withName("INTAKE_DOWN"),

                RobotState.HANDOFF, // your state 2
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Swing LOWERED: " + Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new IntakeSpinCommand(intake, 0),
                        new SpinGroundIntakeCommand(spin, 0.3) // start spin
                    ),
                    
                    

                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        new SpinGroundIntakeCommand(spin, 0.3),
                        new IntakeSpinCommand(intake, 0)
                    ),
                    

                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.ELEVATOR_HANDOFF_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.ELEVATOR_HANDOFF_DELTA.in(Meters))),
                        new SpinGroundIntakeCommand(spin, 0.3),
                        new IntakeSpinCommand(intake, -0.3)
                    ),

                    Commands.deadline(
                        Commands.waitSeconds(1),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.ELEVATOR_HANDOFF_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.ELEVATOR_HANDOFF_DELTA.in(Meters))),
                        new SpinGroundIntakeCommand(spin, -0.5).withTimeout(1),
                        new IntakeSpinCommand(intake, -0.5).withTimeout(1)
                    ),

                    new ParallelCommandGroup(
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )

                ).withName("HANDOFF"),
                
                RobotState.L1,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters)))
                    
                    ),
                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_LEVEL1_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_LEVEL1_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("L1"),
                
                RobotState.L2,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_2_HEIGHT_DELTA.in(Meters)))
                    ),
                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.STAGE_2_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("L2"),

                RobotState.L3,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_3_HEIGHT_DELTA.in(Meters)))
                    
                    ),
                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.STAGE_3_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("L3"),

                RobotState.L4,
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.5),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters)))
                    ),
                    new ParallelCommandGroup(
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    )
                ).withName("L4"),

                RobotState.SCORE4, // a simple canned score if you want a non-parameterized version
                new SequentialCommandGroup(
                    Commands.deadline(
                        Commands.waitSeconds(0.1),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_4_HEIGHT_DELTA.in(Meters))),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    ),
                    Commands.deadline(
                        Commands.waitSeconds(0.3),
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.SCORE_STAGE_4_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0)
                    ),

                    new ParallelCommandGroup(
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.SCORE_STAGE_4_HEIGHT_DELTA.in(Meters))),
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.SCORE_STAGE_4_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.STAGE_4_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0),
                        new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
                    )
                    

                ).withName("SCORE"),

                RobotState.SCOREL1,
                new SequentialCommandGroup(
                    new SpinGroundIntakeCommand(spin, -0.3).withTimeout(1.0)
                ).withName("SCOREL1")
                
            );

            return new SelectCommand<RobotState>(states, ()->current);
            

        
    }
}
