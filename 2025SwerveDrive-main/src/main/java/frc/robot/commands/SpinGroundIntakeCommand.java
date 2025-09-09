package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;


public class SpinGroundIntakeCommand extends Command{
    private final GroundIntakeSubsystem intake;

    public SpinGroundIntakeCommand(GroundIntakeSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
        System.out.println("Ground intake HOLD initialized");
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        System.out.println("HOLDING HOLDING WOWW");
       intake.holdPositionWrite(intake.READPositionPoint());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
