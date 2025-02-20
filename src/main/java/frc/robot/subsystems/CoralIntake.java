package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.CoralIntakeConfig;
import frc.robot.Constants.CoralIntakeCANIds;

public class CoralIntake extends SubsystemBase{

    private TalonFX m_motor = new TalonFX(CoralIntakeCANIds.kCoralIntake);

    private double m_velo = 0.0;
    private boolean m_enabled = false;

    private MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(m_velo);

    
    public CoralIntake(){

        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.getConfigurator().apply(new CoralIntakeConfig(60.0, 55.0, 400.0, 4000.0, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake));

    }

    public void setVelo(double velo){
        m_velo = velo;
    }
    
    public Command setVeloCMD(double velo){
        return new InstantCommand(()-> setVelo(velo));
    }

    public void enable(){
        m_enabled = true;
    }

    public Command enableCMD(){
        return new InstantCommand(()-> enable());
    }

    public void enable(double velo){
        setVelo(velo);
        m_enabled = true;
    }

    public Command enableCMD(double velo){
        return new InstantCommand(()-> enable(velo));
    }

    public void disable(){
        m_enabled = false;
    }

    @Override
    public void periodic() {
        if(m_enabled){
            m_motor.setControl(m_request.withVelocity(m_velo));
        } else {
            m_motor.stopMotor();
        }

        StatusSignal<AngularVelocity> velo = m_motor.getVelocity().refresh();
        SmartDashboard.putNumber("Coral Wrist Velo.", velo.getValueAsDouble());

        StatusSignal<AngularAcceleration> acc = m_motor.getAcceleration().refresh();
        SmartDashboard.putNumber("Coral Wrist Acc.", acc.getValueAsDouble());
    }
}

