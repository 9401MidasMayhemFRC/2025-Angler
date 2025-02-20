package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.CoralIntakeWristConfig;
import frc.robot.Constants.CoralIntakeCANIds;

public class CoralIntakeWrist extends SubsystemBase{

    private TalonFX m_motor = new TalonFX(CoralIntakeCANIds.kCoralWrist);

    private boolean m_enabled = true;
    private double m_pose = 0.0;

    private MotionMagicVoltage m_request = new MotionMagicVoltage(m_pose);

    private StatusSignal<Angle> pose = m_motor.getPosition();

    public CoralIntakeWrist(){
        m_motor.setPosition(0.0);
        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.getConfigurator().apply(new CoralIntakeWristConfig(60.0, 55.0, 160.0, 80.0, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake));
    }

    public void setPose(double pose){
        m_pose = pose;
    }
    
    public Command setPoseCMD(double pose){
        return new InstantCommand(()-> setPose(pose));
    }

    public void enable(){
        m_enabled = true;
    }

    public Command enableCMD(){
        return new InstantCommand(()-> enable());
    }

    public void enable(double pose){
        setPose(pose);
        m_enabled = true;
    }

    public Command enableCMD(double pose){
        return new InstantCommand(()-> enable(pose));
    }

    public void disable(){
        m_enabled = false;
    }

    public Command disableCMD(){
       return new InstantCommand(()-> disable()); 
    }
    
    public double getPose(){
        return pose.refresh().getValueAsDouble();
    }

    @Override
    public void periodic() {
        if(m_enabled){
            m_motor.setControl(m_request.withPosition(m_pose));
        } else {
            m_motor.stopMotor();
        }

        SmartDashboard.putNumber("Coral Wrist Pose.", pose.refresh().getValueAsDouble());

        StatusSignal<AngularVelocity> velo = m_motor.getVelocity().refresh();
        SmartDashboard.putNumber("Coral Wrist Velo.", velo.getValueAsDouble());

        StatusSignal<AngularAcceleration> acc = m_motor.getAcceleration().refresh();
        SmartDashboard.putNumber("Coral Wrist Acc.", acc.getValueAsDouble());
    }
}
