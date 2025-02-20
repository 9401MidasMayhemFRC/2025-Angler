// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorCANIds;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.*;

import frc.robot.commands.*;

public class RobotContainer {

    //--------------------------------------------Swerve---------------------------------------------------------------------------------
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed aka 4.87 mps
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double TranslationSlewRate = 1.0;
    private double RotationalSlewRate = 1.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo); // Using Closed loop with MotionMagicExpo
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    //-----------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------Controllers-----------------------------------------------------------------------
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    //-----------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------Subsystems------------------------------------------------------------------------
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public final AlgaeIntake m_algaeIntake = new AlgaeIntake();
    public final AlgaeIntakeWrist m_algaeWrist = new AlgaeIntakeWrist();

    public final CoralIntake m_coralIntake = new CoralIntake();
    public final CoralIntakeWrist m_coralWrist = new CoralIntakeWrist();
    //-----------------------------------------------------------------------------------------------------------------------------

    //--------------------------------------------Triggers-------------------------------------------------------------
    //public Trigger cupholder = new Trigger(Elevator::getCupholder);
    //-----------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------Commands--------------------------------------------------------------------------
    //-----------------------------------------------------------------------------------------------------------------------------

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-m_driverController.getLeftY() * MaxSpeed) * TranslationSlewRate) // Drive forward with negative Y (forward)
                    .withVelocityY((-m_driverController.getLeftX() * MaxSpeed) * TranslationSlewRate) // Drive left with negative X (left)
                    .withRotationalRate((-m_driverController.getRightX() * MaxAngularRate) * RotationalSlewRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_driverController.leftBumper().onTrue(new IntakeCoral(m_coralIntake,m_coralWrist));
        m_driverController.rightBumper().whileTrue(new IntakeAlgae(m_algaeIntake, m_algaeWrist));

        m_driverController.pov(0).onTrue(new TestRobotCommandedMovement(drivetrain, forwardStraight, 0.5, 0.0, 0.0));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}