package frc.robot.subsystems.GroundIntakeSubsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstant;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.Constants.IntakeConstant;
import frc.robot.subsystems.ArmSubsystem.ArmState;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class SpinGroundIntakeSubsystem extends SubsystemBase {
    private TalonFX m_IntakeKraken;

    private PositionDutyCycle m_IntakepidPosition;

    private double setpoint = 0; // Stores the last commanded position

    public SpinGroundIntakeSubsystem(){
        m_IntakeKraken = new TalonFX(GroundIntakeConstants.kGroundIntakeID, GroundIntakeConstants.kGroundIntakeCANbus);

        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.5;
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        m_IntakeKraken.getConfigurator().apply(slot0Configs);



        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = GroundIntakeConstants.kGroundIntakeCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_IntakeKraken.getConfigurator().apply(currentLimitConfigs);

        m_IntakepidPosition = new PositionDutyCycle(0);
        m_IntakeKraken.setNeutralMode(NeutralModeValue.Brake);

        resetEncoder();
    }



    public void resetEncoder() {
        m_IntakeKraken.setPosition(0);
    }

    
    public double getCurrentPosition_Intake() {
        return m_IntakeKraken.getPosition().getValueAsDouble();
    }



    public void setIntake(){
        m_IntakeKraken.set(0.3);

    }

    public void setIntake(double speed){
        m_IntakeKraken.set(speed);

    }

    public void holdPositionWrite(double setpoint){
        m_IntakeKraken.setControl(m_IntakepidPosition.withPosition(setpoint));
    }

    public void holdPositionStore(double setpoint){
        this.setpoint = setpoint;
    }

    public double READPositionPoint(){
        return setpoint; 
    }


    public void stop() {
        m_IntakeKraken.set(0);
    }

    @Override
    public void periodic() {
    }
}
