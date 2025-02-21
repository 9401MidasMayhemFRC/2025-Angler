package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    public static class AlgaeIntakeWristConfig extends TalonFXConfiguration{

    /**
     * @param statorLimit Stator current limit of motor
     * @param supplyLimit Supply current limit of motor
     * @param acc Accerlation of MotionMagic in rps/s
     * @param velo Accerlation of MotionMagic in rps
     * @param ForwardSoftLimit Soft Limit in the forward direction of the motor
     * @param ReverseSoftLimit Soft Limit in the reverse direction of the motor
     */
    public AlgaeIntakeWristConfig(double statorLimit, double supplyLimit, double acc, double velo, double ForwardSoftLimit, double ReverseSoftLimit, InvertedValue inverted, NeutralModeValue stopType){
      this.CurrentLimits = new AlgaeIntakeWristCurrentConfig(statorLimit,supplyLimit);
      this.MotionMagic = new AlgaeIntakeWristMotionMagicConfig(acc, velo);
      this.MotorOutput = new AlgaeIntakeWristMotorOutputConfig(inverted, stopType);
      this.Slot0 = new AlgaeIntakeWrist_PID_Config();
      this.SoftwareLimitSwitch = new AlgaeIntakeWristSoftLimits(ForwardSoftLimit, ReverseSoftLimit);
    }

    private static class AlgaeIntakeWristCurrentConfig extends CurrentLimitsConfigs{

      private AlgaeIntakeWristCurrentConfig(double statorLimit,double supplyLimit){
        this.StatorCurrentLimit = statorLimit;
        this.StatorCurrentLimitEnable = true;
        
        this.SupplyCurrentLimit = supplyLimit;
        this.SupplyCurrentLimitEnable = true;
      }

    }

    private static class AlgaeIntakeWristMotionMagicConfig extends MotionMagicConfigs{

      private AlgaeIntakeWristMotionMagicConfig(double acc, double velo){
        this.MotionMagicAcceleration = acc;
        this.MotionMagicCruiseVelocity = velo;
      }
    }

    private static class AlgaeIntakeWristMotorOutputConfig extends MotorOutputConfigs{

      private AlgaeIntakeWristMotorOutputConfig(InvertedValue inverted, NeutralModeValue stopType){
        this.Inverted = inverted;
        this.NeutralMode = stopType;
      }
    }

    private static class AlgaeIntakeWristSoftLimits extends SoftwareLimitSwitchConfigs{
      private AlgaeIntakeWristSoftLimits(double ForwardSoftLimit, double ReverseSoftLimit){
        this.ForwardSoftLimitThreshold = ForwardSoftLimit;
        this.ForwardSoftLimitEnable = true;

        this.ReverseSoftLimitThreshold = ForwardSoftLimit;
        this.ReverseSoftLimitEnable = true;
      }
    }

    private static class AlgaeIntakeWrist_PID_Config extends Slot0Configs{
      
      private AlgaeIntakeWrist_PID_Config(){
        this.kS = 0.25; // Add 0.25 V output to overcome static friction
        this.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        this.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        this.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        this.kI = 0; // no output for integrated error
        this.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
      }
    }

  }

  public static class CoralIntakeWristConfig extends TalonFXConfiguration{

    /**
     * @param statorLimit Stator current limit of motor
     * @param supplyLimit Supply current limit of motor
     * @param acc Accerlation of MotionMagic in rps/s
     * @param velo Accerlation of MotionMagic in rps
     * @param ForwardSoftLimit Soft Limit in the forward direction of the motor
     * @param ReverseSoftLimit Soft Limit in the reverse direction of the motor
     */
    public CoralIntakeWristConfig(double statorLimit, double supplyLimit, double acc, double velo, double ForwardSoftLimit, double ReverseSoftLimit, InvertedValue inverted, NeutralModeValue stopType){
      this.CurrentLimits = new CoralIntakeWristCurrentConfig(statorLimit,supplyLimit);
      this.MotionMagic = new CoralIntakeWristMotionMagicConfig(acc, velo);
      this.MotorOutput = new CoralIntakeWristMotorOutputConfig(inverted, stopType);
      this.Slot0 = new CoralIntakeWrist_PID_Config();
      this.SoftwareLimitSwitch = new CoralIntakeWristSoftLimits(ForwardSoftLimit,ReverseSoftLimit);
    }

    private static class CoralIntakeWristCurrentConfig extends CurrentLimitsConfigs{

      private CoralIntakeWristCurrentConfig(double statorLimit,double supplyLimit){
        this.StatorCurrentLimit = statorLimit;
        this.StatorCurrentLimitEnable = true;
        
        this.SupplyCurrentLimit = supplyLimit;
        this.SupplyCurrentLimitEnable = true;
      }

    }

    private static class CoralIntakeWristMotionMagicConfig extends MotionMagicConfigs{

      private CoralIntakeWristMotionMagicConfig(double acc, double velo){
        this.MotionMagicAcceleration = acc;
        this.MotionMagicCruiseVelocity = velo;
      }
    }

    private static class CoralIntakeWristMotorOutputConfig extends MotorOutputConfigs{

      private CoralIntakeWristMotorOutputConfig(InvertedValue inverted, NeutralModeValue stopType){
        this.Inverted = inverted;
        this.NeutralMode = stopType;
      }
    }

    private static class CoralIntakeWristSoftLimits extends SoftwareLimitSwitchConfigs{
      private CoralIntakeWristSoftLimits(double ForwardSoftLimit, double ReverseSoftLimit){
        this.ForwardSoftLimitThreshold = ForwardSoftLimit;
        this.ForwardSoftLimitEnable = true;

        this.ReverseSoftLimitThreshold = ForwardSoftLimit;
        this.ReverseSoftLimitEnable = true;
      }
    }

    private static class CoralIntakeWrist_PID_Config extends Slot0Configs{
      
      private CoralIntakeWrist_PID_Config(){
        this.kS = 0.25; // Add 0.25 V output to overcome static friction
        this.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        this.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        this.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        this.kI = 0; // no output for integrated error
        this.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
      }
    }

  }

  public static class AlgaeIntakeConfig extends TalonFXConfiguration{

    /**
     * @param statorLimit Stator current limit of motor
     * @param supplyLimit Supply current limit of motor
     * @param acc Target Accerlation of MotionMagic in rps/s
     * @param velo Target jerk of MotionMagic in rps/s/s
     */
    public AlgaeIntakeConfig(double statorLimit, double supplyLimit, InvertedValue inverted, NeutralModeValue stopType){
      this.CurrentLimits = new AlgaeIntakeCurrentConfig(statorLimit,supplyLimit);
      this.MotorOutput = new AlgaeIntakeMotorOutputConfig(inverted, stopType);
      this.Slot0 = new AlgaeIntake_PID_Config();
    }

    private static class AlgaeIntakeCurrentConfig extends CurrentLimitsConfigs{

      private AlgaeIntakeCurrentConfig(double statorLimit,double supplyLimit){
        this.StatorCurrentLimit = statorLimit;
        this.StatorCurrentLimitEnable = true;
        
        this.SupplyCurrentLimit = supplyLimit;
        this.SupplyCurrentLimitEnable = true;
      }

    }

    private static class AlgaeIntakeMotorOutputConfig extends MotorOutputConfigs{

      private AlgaeIntakeMotorOutputConfig(InvertedValue inverted, NeutralModeValue stopType){
        this.Inverted = inverted;
        this.NeutralMode = stopType;
      }
    }

    private static class AlgaeIntake_PID_Config extends Slot0Configs{
      
      private AlgaeIntake_PID_Config(){
        this.kS = 0.25; // Add 0.25 V output to overcome static friction
        this.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        this.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        this.kP = 0.11; // An error of 1 rps results in 0.11 V output
        this.kI = 0; // no output for integrated error
        this.kD = 0; // no output for error derivative
      }
    }
  }

  public static class CoralIntakeConfig extends TalonFXConfiguration{

    /**
     * @param statorLimit Stator current limit of motor
     * @param supplyLimit Supply current limit of motor
     * @param acc Target Accerlation of MotionMagic in rps/s
     * @param velo Target jerk of MotionMagic in rps/s/s
     */
    public CoralIntakeConfig(double statorLimit, double supplyLimit, InvertedValue inverted, NeutralModeValue stopType){
      this.CurrentLimits = new CoralIntakeCurrentConfig(statorLimit,supplyLimit);
      this.MotorOutput = new CoralIntakeMotorOutputConfig(inverted, stopType);
      this.Slot0 = new CoralIntake_PID_Config();
    }

    private static class CoralIntakeCurrentConfig extends CurrentLimitsConfigs{

      private CoralIntakeCurrentConfig(double statorLimit,double supplyLimit){
        this.StatorCurrentLimit = statorLimit;
        this.StatorCurrentLimitEnable = true;
        
        this.SupplyCurrentLimit = supplyLimit;
        this.SupplyCurrentLimitEnable = true;
      }

    }

    private static class CoralIntakeMotorOutputConfig extends MotorOutputConfigs{

      private CoralIntakeMotorOutputConfig(InvertedValue inverted, NeutralModeValue stopType){
        this.Inverted = inverted;
        this.NeutralMode = stopType;
      }
    }

    private static class CoralIntake_PID_Config extends Slot0Configs{
      
      private CoralIntake_PID_Config(){
        this.kS = 0.25; // Add 0.25 V output to overcome static friction
        this.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        this.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        this.kP = 0.11; // An error of 1 rps results in 0.11 V output
        this.kI = 0; // no output for integrated error
        this.kD = 0; // no output for error derivative
      }
    }
  }

}
