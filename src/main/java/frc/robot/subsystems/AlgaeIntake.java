package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.AlgaeIntakeConfig;
import frc.robot.Constants.AlgaeIntakeCANIds;

public class AlgaeIntake extends SubsystemBase {

    private TalonFX m_motor = new TalonFX(AlgaeIntakeCANIds.kAlgaeIntake);

    private double m_volts = 0.0;

    private StatusSignal<Current> amps = m_motor.getStatorCurrent();

    
    public AlgaeIntake(){

        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.getConfigurator().apply(new AlgaeIntakeConfig(80.0, 70.0, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake));

    }

    public void setVolts(double volts){
        m_volts = volts;
    }
    
    public Command setVoltsCMD(double volts){
        return new InstantCommand(()-> setVolts(volts));
    }

    public double getCurrent(){
        return amps.refresh().getValueAsDouble();
    }

    @Override
    public void periodic() {
        m_motor.setVoltage(m_volts);

        StatusSignal<Voltage> volts = m_motor.getMotorVoltage().refresh();
        SmartDashboard.putNumber("Algae Intake Volts", volts.getValueAsDouble());

        StatusSignal<AngularVelocity> velo = m_motor.getVelocity().refresh();
        SmartDashboard.putNumber("Algae Intake Velo.", velo.getValueAsDouble());

        SmartDashboard.putNumber("Algae Intake Current", getCurrent());

        StatusSignal<AngularAcceleration> acc = m_motor.getAcceleration().refresh();
        SmartDashboard.putNumber("Algae Intake Acc.", acc.getValueAsDouble());
    }

}
