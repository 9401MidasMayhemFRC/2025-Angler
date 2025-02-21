// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;

import au.grapplerobotics.LaserCan;
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
    /* Setting up bindings for necessary control of the swerve drive platform */
    public final DriveCommands m_driveCommands = new DriveCommands();

    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    //-----------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------Controllers-----------------------------------------------------------------------
    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    //-----------------------------------------------------------------------------------------------------------------------------

    //-------------------------------------------Subsystems------------------------------------------------------------------------
    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    
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
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
           DriveCommands.fieldOrientedDrive(m_drivetrain, m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX)
        );

        //m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        m_driverController.leftBumper().onTrue(m_coralWrist.setPoseCMD(17.85).alongWith(m_coralIntake.setVoltsCMD(2.5))).onFalse(m_coralWrist.setPoseCMD(17.85).alongWith(m_coralIntake.setVoltsCMD(-0.5)));
        m_driverController.leftTrigger().whileTrue(m_coralIntake.setVoltsCMD(2.5)).onFalse(m_coralIntake.setVoltsCMD(0.0));
        m_driverController.rightBumper().whileTrue(new IntakeAlgae(m_algaeIntake, m_algaeWrist));

        m_drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}