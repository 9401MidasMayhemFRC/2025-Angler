package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TestRobotCommandedMovement extends Command {

    CommandSwerveDrivetrain m_drivetrain;
    SwerveRequest.RobotCentric m_request;

    double x;
    double y;
    double rot;

    boolean m_finished = false;

    Timer m_time = new Timer();

    public TestRobotCommandedMovement(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric request, double xSpeed, double ySpeed, double rotSpeed){

        m_drivetrain = drivetrain;
        m_request = request;

        addRequirements(m_drivetrain);

        x = xSpeed;
        y = ySpeed;
        rot = rotSpeed;

    }

    @Override
    public void initialize() {
        m_finished = false;
        m_drivetrain.applyRequest(()-> m_request.withVelocityX(x).withVelocityY(y).withRotationalRate(rot));
        m_time.reset();
        m_time.start();
    }

    @Override
    public void execute() {
        if(m_time.get() >= 1.0){
            m_finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(()-> m_request.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
    }
    
}
