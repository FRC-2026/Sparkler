package frc.robot;

import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.Intake;

/**
 * Modern 2025–2026 RobotContainer for swerve drive with YAGSL.
 */
public class RobotContainer {

    // Subsystems
    private final Shooter ballSubsystem = new Shooter();
    //private final Intake intake = new Intake();
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_CONTROLLER_PORT);


    // Autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        // Default swerve drive command
        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    new Translation2d(
                        -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband)
                    ),
                    -MathUtil.applyDeadband(driverController.getRightX()*DriveConstants.kMaxAngularSpeed, OIConstants.kDriveDeadband),
                    true,  // field relative
                    false  // closed loop
                ),
                m_robotDrive
            )
        );
    }

    /**
     * Configure controller buttons and triggers.
     */
    private void configureBindings() {
        /* DRIVER CONTROLLER */
        // Start button: zero gyro
        driverController.start()
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroGyro(), m_robotDrive));

        // Left bumper: decrease drivabse speed
        driverController.leftBumper()
            .onTrue(new InstantCommand(() -> m_robotDrive.speedDecrease(), m_robotDrive));

        // Right bumper: increase drivebase speed
        driverController.rightBumper()
            .onTrue(new InstantCommand(() -> m_robotDrive.speedIncrease(), m_robotDrive));

        // B button: optional zero heading (alias for gyro)
        driverController.b()
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroGyro(), m_robotDrive));

        
        /* OPERATOR CONTROLLER */
        // When left bumper is pressed, decrease speed of shooter 
        operatorController.leftBumper()
            .onTrue(new InstantCommand(() -> ballSubsystem.speedDecrease(), ballSubsystem));
        
        // When right bumper is pressed, increase speed of shooter 
        operatorController.rightBumper()
            .onTrue(new InstantCommand(() -> ballSubsystem.speedIncrease(), ballSubsystem));

        // While the left trigger on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop (shooter).
        operatorController.leftTrigger()
            .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
                .andThen(ballSubsystem.launchCommand())
                .finallyDo(() -> ballSubsystem.stop()));

        // // While the A button is held, eject fuel 
        // operatorController.b()
        //     .whileTrue(ballSubsystem.runEnd(() -> intake.eject(), () -> ballSubsystem.stop()));
        
        // // While the Y button on operator controller is held, intake Fuel
        // operatorController.a()
        //     .whileTrue(ballSubsystem.runEnd(() -> intake.intake(), () -> ballSubsystem.stop()));
        
        // // Move intake arm down
        // operatorController.x()
        //     .whileTrue(ballSubsystem.runEnd(() -> intake.intakeArm(), () -> ballSubsystem.stop()));

        // // Move intake arm up 
        // operatorController.y()
        //     .whileTrue(ballSubsystem.runEnd(() -> intake.reverseIntakeArm(), () -> ballSubsystem.stop()));
    }

    /**
     * Called at the start of autonomous to get selected auto command.
     */
    public Command getAutonomousCommand() {
        // Example: reset odometry to starting pose at auto start
        m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        return autoChooser.getSelected();
    }

    /**
     * Return the currently loaded PathPlanner command (if used).
     */
    public Command pathplanner() {
        return autoChooser.getSelected();
    }

    // Helper: read current robot pose from DriveSubsystem
    public Pose2d getRobotPose() {
        return m_robotDrive.getPose();
    }
}