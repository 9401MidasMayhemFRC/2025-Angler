package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntakeWrist;

public class AccuateAlgaeIntake extends Command {

    AlgaeIntake m_intake;
    AlgaeIntakeWrist m_wrist;

    boolean m_finished = false;
    boolean inPose = false;

    double m_poseSetpoint;
    double m_veloSetpoint;

    public AccuateAlgaeIntake(AlgaeIntake intake, AlgaeIntakeWrist wrist, double poseSetpoint, double veloSetpoint){
        m_intake = intake;
        m_wrist = wrist;

        m_poseSetpoint = poseSetpoint;
        m_veloSetpoint = veloSetpoint;

        addRequirements(m_intake,m_wrist);
    }

    @Override
    public void initialize() {

        m_finished = false;
        inPose = false;
        m_wrist.setPose(m_poseSetpoint);
        
    }

    @Override
    public void execute() {

        if(m_wrist.getPose() >= (m_poseSetpoint - 2.5)){
            m_intake.setVelo(m_veloSetpoint);

            inPose = true;
        }

        if(inPose && (m_intake.getCurrent() > 50.0)){
            m_finished = true;
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
