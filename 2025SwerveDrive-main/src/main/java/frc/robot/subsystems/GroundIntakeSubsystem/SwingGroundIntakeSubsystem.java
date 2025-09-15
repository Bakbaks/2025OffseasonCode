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

public class SwingGroundIntakeSubsystem extends SubsystemBase {
    private TalonFX m_SwingKraken;

    private PositionDutyCycle m_SwingpidPosition;

    private double setpoint = 0; // Stores the last commanded position
    private double Swingsetpoint = 0;

    public SwingGroundIntakeSubsystem(){
        m_SwingKraken = new TalonFX(GroundIntakeConstants.kGroundSwingID, GroundIntakeConstants.kGroundIntakeCANbus);

        var talonFXConfigs = new TalonFXConfiguration();
        // set slot 0 gains
    
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = GroundIntakeConstants.kGroundP; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = GroundIntakeConstants.kGroundI; // no output for integrated error
        slot0Configs.kD = GroundIntakeConstants.kGroundD; // A velocity error of 1 rps results in 0.1 V output
        m_SwingKraken.getConfigurator().apply(slot0Configs);

        // current limit
        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = GroundIntakeConstants.kGroundIntakeSwingCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;
        m_SwingKraken.getConfigurator().apply(currentLimitConfigs);

        m_SwingpidPosition = new PositionDutyCycle(0);
        m_SwingKraken.setNeutralMode(NeutralModeValue.Brake);
    }



    public void setGroundIntakeAngle(double targetAngle) {

        double Rotations = (targetAngle/360) * GroundIntakeConstants.GroundIntakeGearRatio;

        Swingsetpoint = Rotations;
        //Swingsetpoint = Math.min(Rotations, GroundIntakeConstants.kMaxAngle);
        //Swingsetpoint = Math.max(Rotations, GroundIntakeConstants.kMinAngle);

        if(Swingsetpoint !=  Rotations){ 
            System.out.println("Warning: Requested Ground Intake angle is out of bounds. Setting to " + setpoint + " rotations");
        }


        System.out.println("GROUND INTAKE TO " + Swingsetpoint + " ROTATIONS");

        m_SwingKraken.setControl(m_SwingpidPosition.withPosition(Swingsetpoint));

    }




   

    
    public double getGroundIntakeAngle_Rotation() {
        return m_SwingKraken.getPosition().getValueAsDouble() / GroundIntakeConstants.GroundIntakeGearRatio;
    }

  
    public double getCurrentPosition_Swing() {
        return m_SwingKraken.getPosition().getValueAsDouble();
    }



    public void holdPositionStore(double setpoint){
        this.setpoint = setpoint;
    }

    public double READPositionPoint(){
        return setpoint; 
    }


    public void stop() {
        m_SwingKraken.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ground Intake Motor Output", m_SwingKraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Ground Intake Rotations", getGroundIntakeAngle_Rotation());
    }
}
