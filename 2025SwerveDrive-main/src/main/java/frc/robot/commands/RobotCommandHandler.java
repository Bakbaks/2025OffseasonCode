package frc.robot.commands;

import frc.robot.commands.GroundIntakeCommands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.*;
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


public class RobotCommandHandler extends Command{

    private final int state;
    private Distance targetPosition = null;
    private Angle angle = null;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem arm;
    private final ClimbSubsystem climb;
    private final IntakeSubsystem intake;
    private final SwingGroundIntakeSubsystem SwinggroundIntake;
    private final SpinGroundIntakeSubsystem SpingroundIntake;

    public RobotCommandHandler(int state, ElevatorSubsystem elevatorSubsystem, 
    ArmSubsystem arm, ClimbSubsystem climb, 
    IntakeSubsystem intake, 
    SwingGroundIntakeSubsystem SwinggroundIntake, SpinGroundIntakeSubsystem SpingroundIntake){

        this.state = state;
        this.elevatorSubsystem = elevatorSubsystem;
        this.arm = arm;
        this.climb = climb;
        this.intake = intake;
        this.SwinggroundIntake = SwinggroundIntake;
        this.SpingroundIntake = SpingroundIntake;

    }

    public RobotCommandHandler(int state, ElevatorSubsystem elevatorSubsystem, 
    ArmSubsystem arm, ClimbSubsystem climb, 
    IntakeSubsystem intake, 
    SwingGroundIntakeSubsystem SwinggroundIntake, SpinGroundIntakeSubsystem SpingroundIntake, 
    Distance targetPosition, Angle angle){

        this.state = state;
        this.elevatorSubsystem = elevatorSubsystem;
        this.arm = arm;
        this.climb = climb;
        this.intake = intake;
        this.SwinggroundIntake = SwinggroundIntake;
        this.SpingroundIntake = SpingroundIntake;
        this.targetPosition = targetPosition;
        this.angle = angle;
    }


    @Override
    public void initialize(){
        // using magic ints are not that good but it is what it is for now
        if(state == 0){ // planning on just robot starting position
            
            // set elevator to default position
            new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default, Height: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Units.Meters))).schedule();

                
            Commands.sequence(
                Commands.waitSeconds(0.50),
                Commands.parallel(
                    // move ground intake to max up position
            new SwingGroundIntakeCommand(SwinggroundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
            .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            // move arm to default position

            new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)))
                            )
            
            ).schedule();
            
        }else if(state == 1){ // robot ground intake down
            // move ground intake down to lowest position
            new SwingGroundIntakeCommand(SwinggroundIntake, Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))
            .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_LOWERED_ANGLE_VERTICAL.in(Degrees))).schedule();
            Commands.sequence(
            
                Commands.waitSeconds(0.50),

                Commands.parallel(
                // set elevator to absolute lowest postion(gotta change old elevator logic then. and new enum state)
                new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.ELEVATOR_BASE_DELTA)
                .alongWith(Commands.print("Elevator lowest, Height: " + Constants.ElevatorConstants.ELEVATOR_BASE_DELTA.in(Units.Meters))),
                // move arm to default position

                // move arm to default position
                new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))),

                new SpinGroundIntakeCommand(SpingroundIntake, 0.3)
                )
            ).schedule();
            
        }else if(state == 2){ // robot ground intake feed to arm on elevator. final state ground intake up and elevator down
             // set elevator to default position
            new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default, Height: " + Constants.ElevatorConstants.STAGE_0_HEIGHT_DELTA.in(Units.Meters))).schedule();

            new SpinGroundIntakeCommand(SpingroundIntake, 0.3).schedule();

            Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.parallel(
                    // move ground intake to max up position
            new SwingGroundIntakeCommand(SwinggroundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
            .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            // move arm to default position

            new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)))
                            )
            ).schedule();


            Commands.sequence(
                Commands.waitSeconds(1.50),
                Commands.parallel(
                    //new SpinGroundIntakeCommand(SpingroundIntake, -0.3).withTimeout(0.5),
                    new IntakeSpinCommand(intake,0.3).withTimeout(0.5)
                )
            ).schedule();

            
            
        }else if(state == 3){ // robot ground intake up. elevator up and arm ready to score
            
             // move ground intake to max up position
            new SwingGroundIntakeCommand(SwinggroundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))).schedule();
            // set elevator to whatever level aux sets it.
            new ElevatorSetPositionCommand(elevatorSubsystem, targetPosition)
                        .alongWith(Commands.print("Elevator scoring, Height: " + targetPosition.in(Units.Meters))).schedule();
            // set arm ready to score
            new ArmSetPositionCommand(arm, angle.in(Degrees))
                        .alongWith(Commands.print("Arm Level, Angles: " + angle.in(Degrees))).schedule();
                        //arm.setState(4); // therer are alot of state changing. gonna have to organize this. prob gonna just do in container instead

        }else if(state == 4){ // robot scoring. robot moves arm and spits out the coral at the same time.
            
                        // set elevator to whatever level aux sets it.
            new ElevatorSetPositionCommand(elevatorSubsystem, targetPosition)
            .alongWith(Commands.print("Elevator scoring, Height: " + targetPosition.in(Units.Meters))).schedule();
            
            new SwingGroundIntakeCommand(SwinggroundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
                .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))).schedule();

                // arm moves through path. intake spins at the same time.
            new ArmSetPositionCommand(arm, angle.in(Degrees))
            .alongWith(Commands.print("Arm Level, Angles: " + angle.in(Degrees))).schedule();


            new IntakeSpinCommand(intake, 0.1)
                .withTimeout(1.0)
                .schedule();
            
        }else if(state == 5){
            // set elevator to default position
            new ElevatorSetPositionCommand(elevatorSubsystem, Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA)
                        .alongWith(Commands.print("Elevator default, Height: " + Constants.ElevatorConstants.STAGE_1_HEIGHT_DELTA.in(Units.Meters))).schedule();

            //new SpinGroundIntakeCommand(SpingroundIntake, 0.3).schedule();

            Commands.sequence(
                Commands.waitSeconds(0.25),
                Commands.parallel(
                    // move ground intake to max up position
            new SwingGroundIntakeCommand(SwinggroundIntake, Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))
            .alongWith(Commands.print("GroundIntake, Angles: " + Constants.GroundIntakeConstants.GroundIntake_FEED_ANGLE_VERTICAL.in(Degrees))),
            // move arm to default position

            new ArmSetPositionCommand(arm, ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees))
                        .alongWith(Commands.print("Arm Base/zero Position, Angles: " + ArmConstant.ARM_BASE_ANGLE_VERTICAL.in(Degrees)))
                            )
            ).schedule();


            Commands.sequence(
                Commands.waitSeconds(1.50),
                Commands.parallel(
                    new SpinGroundIntakeCommand(SpingroundIntake, -0.4).withTimeout(0.5),
                    new IntakeSpinCommand(intake,-0.4).withTimeout(0.5)
                )
            ).schedule();
        }
        else if(state == 6){ // robot now moves the arm and elevator to pick up the algae
            //elevator moves to second lowest position

            // arm rotates to a certain angle
        }else if(state == 7){ // robot is scoring algae
            //elevator moves to highest position

            //arm rotates to 180 deg

        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    
}