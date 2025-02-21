package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaeIntakeWrist;

public class IntakeAlgae extends Command {

    AlgaeIntake m_intake;
    AlgaeIntakeWrist m_wrist;

    boolean m_finished = false;
    boolean inPose = false;

    double m_poseSetpoint = 0.0;
    double m_voltSetpoint = 2.5;

    public IntakeAlgae(AlgaeIntake intake, AlgaeIntakeWrist wrist){
        m_intake = intake;
        m_wrist = wrist;

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
            m_intake.setVolts(m_voltSetpoint);

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

        m_intake.setVolts(0.0);
        m_wrist.setPose(0.0);
        
    }
}
