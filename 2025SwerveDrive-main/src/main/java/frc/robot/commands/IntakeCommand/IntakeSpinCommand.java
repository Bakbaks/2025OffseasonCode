package frc.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpinCommand extends Command {
    private final IntakeSubsystem intake; 
    private final double speed;

    public IntakeSpinCommand(IntakeSubsystem intake){
        this(intake, 0.30);
    }

    public IntakeSpinCommand(IntakeSubsystem intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.manualControl(speed);  // spin
    }

    @Override
    public void end(boolean interrupted) {
        //intake.stop();                // always stop after timeout/race/cancel
    }

    @Override
    public boolean isFinished() {
        return true;                 // time it from the caller (withTimeout / race)
    }
}
