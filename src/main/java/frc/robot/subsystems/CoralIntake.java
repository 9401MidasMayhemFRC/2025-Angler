package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.robot.Configs.CoralIntakeConfig;
import frc.robot.Constants.CoralIntakeCANIds;

public class CoralIntake extends SubsystemBase{

    private TalonFX m_motor = new TalonFX(CoralIntakeCANIds.kCoralIntake);

    private double m_volts = 0.0;

    
    public CoralIntake(){

        m_motor.getConfigurator().apply(new TalonFXConfiguration());
        m_motor.getConfigurator().apply(new CoralIntakeConfig(60.0, 55.0, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake));

    }

    public void setVolts(double volts){
        m_volts = volts;
    }
    
    public Command setVoltsCMD(double volts){
        return new InstantCommand(()-> setVolts(volts));
    }

    @Override
    public void periodic() {
        m_motor.setVoltage(m_volts);

        StatusSignal<Voltage> volts = m_motor.getMotorVoltage().refresh();
        SmartDashboard.putNumber("Coral Intake Volts", volts.getValueAsDouble());

        StatusSignal<Current> amps = m_motor.getStatorCurrent().refresh();
        SmartDashboard.putNumber("Coral Intake Stator Amps", amps.getValueAsDouble());

        StatusSignal<AngularVelocity> velo = m_motor.getVelocity().refresh();
        SmartDashboard.putNumber("Coral Wrist Velo.", velo.getValueAsDouble());

        StatusSignal<AngularAcceleration> acc = m_motor.getAcceleration().refresh();
        SmartDashboard.putNumber("Coral Wrist Acc.", acc.getValueAsDouble());
    }
}

