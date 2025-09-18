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

    public Command build(ElevatorSubsystem elevator,
                         ArmSubsystem arm,
                         SwingGroundIntakeSubsystem swing,
                         SpinGroundIntakeSubsystem spin,
                         IntakeSubsystem intake){


        
            Map<RobotState, Command> states = Map.of(
                RobotState.START_CONFIG,
                new SequentialCommandGroup(
                    new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                    Commands.waitSeconds(0.50),
                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Swing FEED: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)))
                    )
                ).withName("START_CONFIG"),

                RobotState.INTAKE_DOWN,
                new SequentialCommandGroup(
                    new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Swing LOWERED: " + Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))),
                    Commands.waitSeconds(0.50),
                    new ParallelCommandGroup(
                        new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)
                            .alongWith(Commands.print("Elevator BASE: " + Constants.ElevatorConstants.ELEVATOR_BASE_DELTA.in(Meters))),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                            .alongWith(Commands.print("Arm BASE: " + Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),
                        new SpinGroundIntakeCommand(spin, 0.3)
                    )
                ).withName("INTAKE_DOWN"),

                RobotState.HANDOFF, // your state 2
                new SequentialCommandGroup(
                    new ElevatorSetPositionCommand(elevator, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Meters))),
                    new SpinGroundIntakeCommand(spin, 0.3), // start spin
                    Commands.waitSeconds(0.25),
                    new ParallelCommandGroup(
                        new SwingGroundIntakeCommand(swing, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees)),
                        new ArmSetPositionCommand(arm, Constants.ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                    ),
                    // same-command-twice with different timing: keep your timing
                    Commands.waitSeconds(1.50),
                    new ParallelCommandGroup(
                        // previously: SpinGroundIntake -0.3 optional
                        new IntakeSpinCommand(intake, 0.3).withTimeout(0.5)
                    )
                ).withName("HANDOFF"),

                RobotState.SCORE, // a simple canned score if you want a non-parameterized version
                new SequentialCommandGroup(
                    new IntakeSpinCommand(intake, 0.1).withTimeout(1.0)
                ).withName("SCORE")
            );

            return new SelectCommand<RobotState>(states, ()->current);
            

        
    }
}
