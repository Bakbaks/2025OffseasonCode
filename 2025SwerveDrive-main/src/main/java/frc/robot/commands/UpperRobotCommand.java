package frc.robot.commands;

import frc.robot.commands.GroundIntakeCommands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.ElevatorCommand.*;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstant;
import frc.robot.commands.ArmCommand.*;
import frc.robot.commands.IntakeCommand.*;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.*;


public class UpperRobotCommand extends Command{

    private final int state;
    private Distance targetPosition = null;
    private Angle angle = null;
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();


    public UpperRobotCommand(int state){
        this.state = state;
    }

    public UpperRobotCommand(int state, Distance targetPosition, Angle angle){
        this.state = state;
        this.targetPosition = targetPosition;
        this.angle = angle;
    }


    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(state == 0){ // planning on just robot starting position
            // set elevator to default position
            new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default, Height: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Units.Meters))).schedule();
            // move ground intake to max up position
            new SwingGroundIntakeCommand(groundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))).schedule();
                        groundIntake.setState(1);
            // move arm to default position

            new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)));
        }else if(state == 1){ // robot ground intake down
            // set elevator to absolute lowest postion(gotta change old elevator logic then. and new enum state)
            new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)
                        .alongWith(Commands.print("Elevator lowest, Height: " + Constants.ElevatorConstants.ELEVATOR_BASE_DELTA.in(Units.Meters))).schedule();
            // move ground intake down to lowest position
            new SwingGroundIntakeCommand(groundIntake, Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))).schedule();
                        groundIntake.setState(2);
            // move arm to default position
            new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)));
        }else if(state == 2){ // robot ground intake feed to arm on elevator. final state ground intake up and elevator down
            // set elevator to second lowest position
            new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default, Height: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Units.Meters))).schedule();
    
            //set ground intake to max up position
            new SwingGroundIntakeCommand(groundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                                .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))).schedule();
                        groundIntake.setState(1);
            // move arm to default position
            new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)));
            // spin ground intake
            Command spin1 = new SpinGroundIntakeCommand(groundIntake); // somehow make it spin for 1 second
            //spin arm motors

            Command spin2 = new IntakeHoldPositionCommand(intake);

            // Run the two spin commands and the wait command in parallel.
            // The race group will end as soon as the waitCommand finishes,
            // interrupting the spin commands.
            Command waitCommand = new WaitCommand(1);
            new ParallelRaceGroup(spin1, spin2, waitCommand);
        }else if(state == 3){ // robot ground intake up. elevator up and arm ready to score
            // set elevator to whatever level aux sets it.
            new ElevatorSetPositionCommand(elevatorSubsystem, targetPosition)
                        .alongWith(Commands.print("Elevator scoring, Height: " + targetPosition.in(Units.Meters))).schedule();
            // set arm ready to score, 5 degrees off from actual angle
            new ArmSetPositionCommand(arm, angle.in(Degrees))
                        .alongWith(Commands.print("Arm Level, Angles: " + angle.in(Degrees))).schedule();
                        //arm.setState(4); // therer are alot of state changing. gonna have to organize this. prob gonna just do in container instead

        }else if(state == 4){ // robot scoring. robot moves arm and spits out the coral at the same time.
            // arm moves through path. intake spins at the same time.
        }else if(state == 5){ // robot now moves the arm and elevator to pick up the algae
            //elevator moves to second lowest position

            // arm rotates to a certain angle
        }else if(state == 6){ // robot is scoring algae
            //elevator moves to highest position

            //arm rotates to 180 deg

        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    
}
