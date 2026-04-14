package frc.robot;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LauncherSubsystem;

/**
 * Modern 2025–2026 RobotContainer for swerve drive with YAGSL.
 */
public class RobotContainer {
    
    // Subsystems
    private final Flywheel ballSubsystem = new Flywheel();
    private final LauncherSubsystem launchSubsystem = new LauncherSubsystem();
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
        autoChooser.setDefaultOption("Default Auto", new InstantCommand(() -> new PathPlannerAuto("Starting Point 2, Shoot").schedule()));
        autoChooser.setDefaultOption("Collect 1", new InstantCommand(() -> new PathPlannerAuto("Starting Point 1, Collect").schedule()));
        autoChooser.setDefaultOption("Collect 1, Shoot", new InstantCommand(() -> new PathPlannerAuto("Starting Point 1, Collect, Shoot").schedule()));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("i", intakeSubsystem.intakeCommand());
        NamedCommands.registerCommand("e", intakeSubsystem.ejectCommand());
        NamedCommands.registerCommand("l", launchSubsystem.launchCommand());
        NamedCommands.registerCommand("sUp", ballSubsystem.spinUpCommand());
        NamedCommands.registerCommand("iAD", intakeSubsystem.indexArmCommand());
        NamedCommands.registerCommand("iAU", intakeSubsystem.reverseIndexArmCommand());


        //Intake commands for auto/pathplanner
        NamedCommands.registerCommand(
            "intake",
            new ParallelDeadlineGroup(
                new WaitCommand(2.5), // how long you want the intake to run
                new StartEndCommand(
                    () -> intakeSubsystem.intake(),  // start intaking
                    () -> intakeSubsystem.stopIntake(),    // stop motors at the end
                    intakeSubsystem                  // the subsystem the command requires
                )
            )
        );
        NamedCommands.registerCommand(
            "indexer",
            new ParallelDeadlineGroup(
                new WaitCommand(2), //Assumuption CHANGE LATER
                new StartEndCommand(
                    () -> intakeSubsystem.indexDown(),               // runs index 
                    () -> intakeSubsystem.stopIntakeArm(),    // stops index 
                    intakeSubsystem
                )
            )
        );
        NamedCommands.registerCommand(
            "intakeArmUp",
            new ParallelDeadlineGroup(
                new WaitCommand(2),
                new StartEndCommand(
                    () -> intakeSubsystem.reverseIndexArm(),        // move arm up
                    () -> intakeSubsystem.stopIntakeArm(),    // stop arm motor
                    intakeSubsystem
                )
            )
        );
        

        // //Shoot commands for auto/pathplanner
        // NamedCommands.registerCommand(
        //     "shootMin",
        //     new StartEndCommand(
        //         () -> {
        //             ballSubsystem.runFlywheel();
        //             launchSubsystem.launchReverse();
        //         },
        //         () -> ballSubsystem.stop(),
        //         ballSubsystem
        //     )
        // );

        NamedCommands.registerCommand(
            "onlyLaunch",
            new StartEndCommand(
                () -> {
                    launchSubsystem.launchReverse();
                },
                () -> launchSubsystem.stopLaunch(),
                launchSubsystem
            )
        );

        NamedCommands.registerCommand(
            "onlyFlywheel",
            new StartEndCommand(
                () -> {
                    ballSubsystem.runFlywheel();
                },
                () -> ballSubsystem.stopFlywheel(),
                ballSubsystem
            )
        );

        // NamedCommands.registerCommand(
        //     "shootComplex",
        //     ballSubsystem.spinUpCommand()

        //         // Small anti-shoot pulse at start
        //         .alongWith(ballSubsystem.reverseLaunchCommand().withTimeout(0.3))
        //         .alongWith(intakeSubsystem.indexArmCommand().withTimeout(0.3))

        //         // Continuous feeding once at speed
        //         .alongWith(
        //             intakeSubsystem.reverseIndexArmCommand()
        //                 .onlyIf(() -> ballSubsystem.atSpeed())
        //         )
        //         .alongWith(
        //             ballSubsystem.launchCommand()
        //                 .onlyIf(() -> ballSubsystem.atSpeed())
        //         )

        //         // Cleanup when command ends
        //         .finallyDo(() -> {
        //             ballSubsystem.stop();
        //             intakeSubsystem.stopIndexArm();
        //         })
        // );

        NamedCommands.registerCommand(
            "shootMax",
            new ParallelDeadlineGroup(
                new WaitCommand(9), //Shoots for 9 (with 10-16 balls)
                new StartEndCommand(
                    () -> ballSubsystem.runFlywheel(),
                    () -> ballSubsystem.stopFlywheel(),
                    ballSubsystem
                )
            )
        );
    }

    // parallel deadline group wait command to choose when it ends sequential command group and then a wait command 
    // run commands in sequence wait a sec then run the other command 

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
            .onTrue(new InstantCommand(() -> ballSubsystem.speedDecrease(100,-10), ballSubsystem));

        operatorController.rightBumper()
            .onTrue(new InstantCommand(() -> ballSubsystem.speedIncrease(100,-5800), ballSubsystem));


        // operatorController.rightTrigger()//launcher
        //     .whileTrue(
        //         Commands.deadline(
        //         Commands.sequence(
        //             Commands.waitSeconds(2), //wait 2 second before running the reverseLanchCommand
        //             Commands.run(() -> ballSubsystem.launchCommand(), ballSubsystem)
        //         ),
        //         Commands.run(() -> ballSubsystem.reverseLaunchCommand(), ballSubsystem),
        //         Commands.run(() -> ballSubsystem.spinUpCommand(), ballSubsystem)
        //     )
        //     ).onFalse(
        //         Commands.runOnce(() -> {
        //             ballSubsystem.stopSpinUP();
        //             ballSubsystem.stopLaunch();
        //     })
        // );

            // ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            //     .andThen(ballSubsystem.launchCommand())
            //     .finallyDo(() -> ballSubsystem.stop()));

        operatorController.leftTrigger()
        .whileTrue(ballSubsystem.spinUpCommand()
        .finallyDo(() -> ballSubsystem.stop()));

        operatorController.rightTrigger()
        .whileTrue(launchSubsystem.reverseLaunchCommand()
        .finallyDo(() -> launchSubsystem.stopLaunch()));


        // operatorController.x()
        // .whileTrue(ballSubsystem.spinUpCommand())
        // .whileTrue(intakeSubsystem.indexArmCommand().withTimeout(2)
        // .andThen(ballSubsystem.reverseLaunchCommand())
        // .finallyDo(() -> ballSubsystem.stop())
        // .finallyDo(() -> intakeSubsystem.stopIndexArm()));

        // operatorController.x()
        // .whileTrue(
        //     // Always spin up flywheel
        //     ballSubsystem.spinUpCommand()

        //     // Small anti-shoot pulse at start
        //     .alongWith(ballSubsystem.reverseLaunchCommand().withTimeout(0.3))
        //     //.alongWith(intakeSubsystem.indexArmCommand().withTimeout(0.3))

        //     // Continuous feeding once at speed
        //     .alongWith(intakeSubsystem.reverseIndexArmCommand().onlyIf(() -> ballSubsystem.atSpeed()))
        //    // .alongWith(ballSubsystem.launchCommand().onlyIf(() -> ballSubsystem.atSpeed()))

        //     .finallyDo(() -> {
        //         ballSubsystem.stop();
        //         intakeSubsystem.stopIndexArm();
        //     })
        // );


        // .whileTrue(
        //     new RunCommand(() -> {
        //         ballSubsystem.runFlywheel();
        //         ballSubsystem.launchCommand();
        //     }, ballSubsystem)
        // )
        // .onFalse(
        //     new InstantCommand(() -> ballSubsystem.stopFlywheel(), ballSubsystem)
        // );

        // this intakes
        operatorController.b()
             .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.eject(), () -> intakeSubsystem.stopRoller()));

        // this ejects... confusing lol
        operatorController.a()
             .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.intake(), () -> intakeSubsystem.stopRoller()));
 
        // makes arm go down
        operatorController.x()
             .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.indexDown(), () -> intakeSubsystem.stopIndexArm()));

        // move index to roll down
        operatorController.y()
            .whileTrue(intakeSubsystem.runEnd(() -> intakeSubsystem.reverseIndexArm(), () -> intakeSubsystem.stopIndexArm()));
    }

    /**
     * Called at the start of autonomous to get selected auto command.
     */
    public Command getAutonomousCommand() {
        // m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
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