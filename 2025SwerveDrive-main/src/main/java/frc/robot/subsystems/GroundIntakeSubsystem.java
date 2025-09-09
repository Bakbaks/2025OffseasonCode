package frc.robot.subsystems;

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

public class GroundIntakeSubsystem extends SubsystemBase {
    private TalonFX m_IntakeKraken;
    private TalonFX m_SwingKraken;

    private PositionDutyCycle m_IntakepidPosition;
    private PositionDutyCycle m_SwingpidPosition;

    private double setpoint = 0; // Stores the last commanded position
    private double Swingsetpoint = 0;
    private int state = 0;
    private GroundIntakeState currentState = GroundIntakeState.UNKNOWN;

    public GroundIntakeSubsystem(){
        m_IntakeKraken = new TalonFX(GroundIntakeConstants.kGroundIntakeID, GroundIntakeConstants.kGroundIntakeCANbus);
        m_SwingKraken = new TalonFX(GroundIntakeConstants.kGroundSwingID, GroundIntakeConstants.kGroundIntakeCANbus);

        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.5;
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        m_IntakeKraken.getConfigurator().apply(slot0Configs);

        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kP = GroundIntakeConstants.kGroundP; // A position error of 2.5 rotations results in 12 V output
        slot1Configs.kI = GroundIntakeConstants.kGroundI; // no output for integrated error
        slot1Configs.kD = GroundIntakeConstants.kGroundD; // A velocity error of 1 rps results in 0.1 V output
        m_SwingKraken.getConfigurator().apply(slot1Configs);

        // current limit
        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = GroundIntakeConstants.kGroundIntakeSwingCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_SwingKraken.getConfigurator().apply(currentLimitConfigs);

        var currentLimitConfigs2 = new CurrentLimitsConfigs();
        currentLimitConfigs2.StatorCurrentLimit = GroundIntakeConstants.kGroundIntakeCurrentLimit;
        currentLimitConfigs2.StatorCurrentLimitEnable = true;
        m_IntakeKraken.getConfigurator().apply(currentLimitConfigs2);

        m_IntakepidPosition = new PositionDutyCycle(0);
        m_IntakeKraken.setNeutralMode(NeutralModeValue.Brake);

        m_SwingpidPosition = new PositionDutyCycle(0);
        m_SwingKraken.setNeutralMode(NeutralModeValue.Brake);
        resetEncoder();
    }

    public enum GroundIntakeState{
        BASE, FEED, LOWERED, UNKNOWN
    }

    public GroundIntakeState getStateE() {
     return currentState;
    }

    public void setState(GroundIntakeState newState) {
        currentState = newState;
    }

    public void setGroundIntakeAngle(double targetAngle) {

        double Rotations = (targetAngle/360) * GroundIntakeConstants.GroundIntakeGearRatio;

        Swingsetpoint = Math.min(Rotations, GroundIntakeConstants.kMaxAngle);
        Swingsetpoint = Math.max(Rotations, GroundIntakeConstants.kMinAngle);

        if(Swingsetpoint !=  Rotations){
            System.out.println("Warning: Requested Ground Intake angle is out of bounds. Setting to " + setpoint + " rotations");
        }


        System.out.println("Setting ground intake pos to final " + Rotations + " rotations");

        m_SwingKraken.setControl(m_SwingpidPosition.withPosition(Swingsetpoint));

    }

    public void setState(int newState){
        state = newState;
    }

    public int getState(){
        return state;
    }


    public void resetEncoder() {
        m_IntakeKraken.setPosition(0);
    }

    
    public double getGroundIntakeAngle_Rotation() {
        return m_SwingKraken.getPosition().getValueAsDouble() / GroundIntakeConstants.GroundIntakeGearRatio;
    }

    public double getCurrentPosition_Intake() {
        return m_IntakeKraken.getPosition().getValueAsDouble();
    }

    public double getCurrentPosition_Swing() {
        return m_SwingKraken.getPosition().getValueAsDouble();
    }

    public void stopIntake() {
        m_IntakeKraken.set(0);
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
        m_SwingKraken.set(0);
        m_IntakeKraken.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Intake Motor Output", m_SwingKraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Ground Intake Degrees", getGroundIntakeAngle_Rotation());
    }
}
