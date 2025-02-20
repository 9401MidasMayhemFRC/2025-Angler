package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralIntakeCANIds;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralIntakeWrist;

public class IntakeCoral extends Command{

    private final double m_intakePoseSetpoint = 0.0;
    private final double m_intakeVeloSetpoint = 15.0;

    //private double m_elevatorPoseSetpoint = 0.0;
    //private double m_clawPoseSetpoint = 0.0;

    private boolean m_finished = false;
    private boolean hasObject = false;

    private CoralIntake m_intake;
    private CoralIntakeWrist m_wrist;
    private LaserCan m_sensor = new LaserCan(CoralIntakeCANIds.kCoralLaserCAN);
    // private Elevator m_elevator;
    // private Claw m_claw;

    public IntakeCoral(CoralIntake intake, CoralIntakeWrist wrist/*, Elevator elevator, Claw claw */){
        m_intake = intake;
        m_wrist = wrist;
        //m_elevator = elevator;
        //m_claw = claw;
        addRequirements(m_intake,m_wrist/*, m_elevator, m_claw */);
        try{
            m_sensor.setRangingMode(LaserCan.RangingMode.SHORT);
            m_sensor.setRegionOfInterest(new LaserCan.RegionOfInterest(16, 10, 16, 16));
            m_sensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
        } catch (Exception e){
            DriverStation.reportError("Coral Intake LaserCAN failed configuration:" + e.getLocalizedMessage(), false);
        }
    }

    @Override
    public void initialize() {

        //set elevator to pose
        //set claw to pose
        m_finished = false;
        hasObject = false;
        m_wrist.setPose(m_intakePoseSetpoint);
        m_intake.setVelo(m_intakeVeloSetpoint);
        
    }

    @Override
    public void execute() {
        Measurement measurement = m_sensor.getMeasurement();
        if((measurement.distance_mm < 440.0) && (measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) && !hasObject){
            m_wrist.setPose(0.0);
            m_intake.setVelo(0.0);
            hasObject = true;
        }

        if(hasObject && (m_wrist.getPose() <= 2.5)){
            m_intake.setVelo(m_intakeVeloSetpoint);

            if(measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT){
                m_finished = true;
            }


        }
        
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setVelo(0.0);
        m_wrist.setPose(0.0);
    }
    
}
