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
import frc.robot.Configs.AlgaeIntakeWristConfig;
import frc.robot.Constants.AlgaeIntakeCANIds;

public class AlgaeIntakeWrist extends SubsystemBase{

    private TalonFX m_motor = new TalonFX(AlgaeIntakeCANIds.kAlgaeWrist);

    private double m_pose = 0.0;
    private StatusSignal<Angle> pose = m_motor.getPosition();

    private MotionMagicVoltage m_request = new MotionMagicVoltage(m_pose); 

    public AlgaeIntakeWrist(){

        m_motor.setPosition(0.0);
        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.getConfigurator().apply(new AlgaeIntakeWristConfig(100.0,95.0,160.0,80.0,100.0,-0.5,InvertedValue.Clockwise_Positive,NeutralModeValue.Brake));

    }

    public void setPose(double pose){
        m_pose = pose;
    }

    public double getPose(){
        return pose.refresh().getValueAsDouble();
    }

    public Command setposeCMD (double pose){
        return new InstantCommand(()-> setPose(pose));
    }

    @Override
    public void periodic() {
        m_motor.setControl(m_request.withPosition(m_pose));

        StatusSignal<Angle> pose = m_motor.getPosition().refresh();
        SmartDashboard.putNumber("Algae wrist position", pose.getValueAsDouble());

        StatusSignal<AngularVelocity> velo = m_motor.getVelocity().refresh();
        SmartDashboard.putNumber("Algae wrist velo", velo.getValueAsDouble());

        StatusSignal<AngularAcceleration> acc = m_motor.getAcceleration().refresh();
        SmartDashboard.putNumber("Algae wrist acc", acc.getValueAsDouble());
    }

}
