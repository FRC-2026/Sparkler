package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Modern 2025–2026 RobotContainer for swerve drive with YAGSL.
 */
public class RobotContainer {

    // Subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // Controllers
    private final CommandXboxController driverController =
            new CommandXboxController(OIConstants.kDriverControllerPort);

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
                    -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                    false,  // field relative
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
        // Start button: zero gyro
        driverController.start()
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroGyro(), m_robotDrive));

        // Left bumper: decrease speed
        driverController.leftBumper()
            .onTrue(new InstantCommand(() -> m_robotDrive.speedDecrease(), m_robotDrive));

        // Right bumper: increase speed
        driverController.rightBumper()
            .onTrue(new InstantCommand(() -> m_robotDrive.speedIncrease(), m_robotDrive));

        // B button: optional zero heading (alias for gyro)
        driverController.b()
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroGyro(), m_robotDrive));

        // Left trigger > 20%: set wheels into X-lock for resisting pushes
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