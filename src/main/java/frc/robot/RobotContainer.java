package frc.robot;

import static frc.robot.Constants.FuelConstants.SPIN_UP_SECONDS;
import static frc.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OperatorConstants.OPERATOR_CONTROLLER_PORT;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

/**
 * Modern 2025–2026 RobotContainer for swerve drive with YAGSL.
 */
public class RobotContainer {
    
    // Subsystems
    private final Shooter ballSubsystem = new Shooter();
    private final Intake intakeSubsystem = new Intake();
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
                        -MathUtil.applyDeadband(driverController.getLeftY()*Math.PI, OIConstants.kDriveDeadband),//why Math.pi??? 
                        -MathUtil.applyDeadband(driverController.getLeftX()*Math.PI, OIConstants.kDriveDeadband)
                    ),
                    -MathUtil.applyDeadband(driverController.getRightX()*Math.PI, OIConstants.kDriveDeadband),
                    true,  // field relative
                    false  // closed loop
                ),
                m_robotDrive));

        // === FIX: Wrap PathPlannerAuto in InstantCommand for dashboard visibility ===
        autoChooser.setDefaultOption("Default Auto", new InstantCommand(() -> new PathPlannerAuto("DefaultAuto").schedule()));
        autoChooser.addOption("My Auto", new InstantCommand(() -> new PathPlannerAuto("MyAuto").schedule()));
        autoChooser.addOption("Starting Point 3, Taxi", new InstantCommand(() -> new PathPlannerAuto("Start Point 3, Taxi").schedule()));
        autoChooser.addOption("Starting Point 3, Shoot", new InstantCommand(() -> new PathPlannerAuto("Start Point 3, Shoot").schedule()));
        autoChooser.addOption("Starting Point 3, Shoot, Collect, Shoot", new InstantCommand(() -> new PathPlannerAuto("Starting Point 3, Shoot, Collect, Shoot").schedule()));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("i", intakeSubsystem.intakeCommand());
        NamedCommands.registerCommand("e", intakeSubsystem.ejectCommand());
        NamedCommands.registerCommand("l", ballSubsystem.launchCommand());
        NamedCommands.registerCommand("sUp", ballSubsystem.spinUpCommand());
        NamedCommands.registerCommand("iAD", intakeSubsystem.intakeArmCommand());
        NamedCommands.registerCommand("iAU", intakeSubsystem.reverseIntakeArmCommand());


        //Intake commands for auto/pathplanner
        NamedCommands.registerCommand(
            "intake",
            new ParallelDeadlineGroup(
                new WaitCommand(2), // how long you want the intake to run
                new StartEndCommand(
                    () -> intakeSubsystem.intake(),  // start intaking
                    () -> intakeSubsystem.stopIntake(),    // stop motors at the end
                    intakeSubsystem                  // the subsystem the command requires
                )
            )
        );
        NamedCommands.registerCommand(
            "intakeArmDown",
            new ParallelDeadlineGroup(
                new WaitCommand(2), //Assumuption CHANGE LATER
                new StartEndCommand(
                    () -> intakeSubsystem.intakeArm(),               // move arm down
                    () -> intakeSubsystem.stopIntakeArm(),    // stop arm motor
                    intakeSubsystem
                )
            )
        );
        NamedCommands.registerCommand(
            "intakeArmUp",
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new StartEndCommand(
                    () -> intakeSubsystem.reverseIntakeArm(),        // move arm up
                    () -> intakeSubsystem.stopIntakeArm(),    // stop arm motor
                    intakeSubsystem
                )
            )
        );
        

        //Shoot commands for auto/pathplanner
        NamedCommands.registerCommand(
            "shootMin",
            new ParallelDeadlineGroup(
                new WaitCommand(5), //Shoots for 5 (for Starting with 8)
                new StartEndCommand(
                    () -> ballSubsystem.spinUp(),
                    () -> ballSubsystem.stop(),
                    ballSubsystem
                )
            )
        );

        NamedCommands.registerCommand(
            "shootMax",
            new ParallelDeadlineGroup(
                new WaitCommand(9), //Shoots for 9 (with 10-16 balls)
                new StartEndCommand(
                    () -> ballSubsystem.spinUp(),
                    () -> ballSubsystem.stop(),
                    ballSubsystem
                )
            )
        );
    }

    /**
     * Configure controller buttons and triggers.
     */
    private void configureBindings() {
        /* DRIVER CONTROLLER */
        driverController.start()
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroGyro(), m_robotDrive));

        driverController.leftBumper()
            .onTrue(new InstantCommand(() -> m_robotDrive.speedDecrease(), m_robotDrive));

        driverController.rightBumper()
            .onTrue(new InstantCommand(() -> m_robotDrive.speedIncrease(), m_robotDrive));

        driverController.b()
            .onTrue(new InstantCommand(() -> m_robotDrive.zeroGyro(), m_robotDrive));

        /* OPERATOR CONTROLLER */
        operatorController.leftBumper()
            .onTrue(new InstantCommand(() -> ballSubsystem.speedDecrease(), ballSubsystem));

        operatorController.rightBumper()
            .onTrue(new InstantCommand(() -> ballSubsystem.speedIncrease(), ballSubsystem));

        operatorController.leftTrigger()
            .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
                .andThen(ballSubsystem.launchCommand())
                .finallyDo(() -> ballSubsystem.stop()));

        operatorController.b()
             .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.eject(), () -> intakeSubsystem.stopRoller()));

        operatorController.a()
             .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.intake(), () -> intakeSubsystem.stopRoller()));

        // makes arm go down
        operatorController.x()
             .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.intakeArm(), () -> intakeSubsystem.stopArm()));

        // makes arm go up
        operatorController.y()
            .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.reverseIntakeArm(), () -> intakeSubsystem.stopArm()));
    }

    /**
     * Called at the start of autonomous to get selected auto command.
     */
    public Command getAutonomousCommand() {
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