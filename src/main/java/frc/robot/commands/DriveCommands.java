package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

public class DriveCommands {
    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed aka 4.87 mps
        private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
            
            
                private static final SwerveRequest.FieldCentric DRIVE_REQUEST = new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * OperatorConstants.kDeadband)
                .withRotationalDeadband(MaxAngularRate * OperatorConstants.kDeadband)
        .withDriveRequestType(DriveRequestType.Velocity);
    private static SlewRateLimiter m_slewX = new SlewRateLimiter(7.0);
    private static SlewRateLimiter m_slewY = new SlewRateLimiter(7.0);
    private static SlewRateLimiter m_slewRot = new SlewRateLimiter(30.0);
    
    public static void updateSlew(double x, double y, double r) {
        m_slewX = new SlewRateLimiter(x, -x, m_slewX.lastValue());
        m_slewY = new SlewRateLimiter(y, -y, m_slewY.lastValue());
        m_slewRot = new SlewRateLimiter(r, -r, m_slewRot.lastValue());
    }
        
    public static Command fieldOrientedDrive(
                    CommandSwerveDrivetrain drivetrain,
                    DoubleSupplier forwardBack,
                    DoubleSupplier leftRight,
                    DoubleSupplier rotation) {

                return drivetrain.applyRequest(() ->
                    DRIVE_REQUEST
                        .withVelocityX(m_slewX.calculate(-forwardBack.getAsDouble() * MaxSpeed))
                        .withVelocityY(m_slewY.calculate(-leftRight.getAsDouble() * MaxSpeed))
                        .withRotationalRate(m_slewRot.calculate(-rotation.getAsDouble() * MaxAngularRate))
        );
    }

    public static Command resetFieldOrientation(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.runOnce(drivetrain::seedFieldCentric);
    }
}