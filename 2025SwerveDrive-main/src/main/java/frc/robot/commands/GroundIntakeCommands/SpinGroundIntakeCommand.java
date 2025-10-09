package frc.robot.commands.GroundIntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem.SpinGroundIntakeSubsystem;


public class SpinGroundIntakeCommand extends Command{
    private final SpinGroundIntakeSubsystem intake;
    private final double speed;

    public SpinGroundIntakeCommand(SpinGroundIntakeSubsystem intake){
        this(intake, 0.30);
    }

    public SpinGroundIntakeCommand(SpinGroundIntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted){
        //intake.stop();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
