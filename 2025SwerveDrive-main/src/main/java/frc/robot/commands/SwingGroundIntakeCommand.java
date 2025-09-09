package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

public class SwingGroundIntakeCommand extends Command{
    private final GroundIntakeSubsystem Intake;
    private final double targetPositionRotations;

    public SwingGroundIntakeCommand(GroundIntakeSubsystem Intake, double targetPosition) {
        this.Intake = Intake;
        this.targetPositionRotations = targetPosition;
        addRequirements(Intake);
    }

    @Override
    public void initialize() {
        System.out.println("Intake initialized");        
        Intake.setGroundIntakeAngle(targetPositionRotations);

    }

    @Override
    public void execute(){
//        System.out.println("--------------------------Target"+targetPositionRotations);
        System.out.println("---------------------------current"+Intake.getGroundIntakeAngle_Rotation()+"my ideal postition: "+targetPositionRotations);
    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
