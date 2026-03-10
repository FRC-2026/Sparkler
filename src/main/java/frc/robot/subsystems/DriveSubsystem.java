package frc.robot.subsystems;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

//red sparkmax
//turning speed
//arm intake, pid


//paths during comp 
//vision subsytem
//

public class DriveSubsystem extends SubsystemBase {

    private SwerveDrive swerveDrive;

    // Feedforward object for swerve drive wheels
    private final SimpleMotorFeedforward driveFeedforward;

    public DriveSubsystem() {
        try {
            File swerveDir = new File(Filesystem.getDeployDirectory(), "swerve");

            swerveDrive = new SwerveParser(swerveDir)
                    .createSwerveDrive(Constants.maxspeed);

            // Initialize feedforward constants from ModuleConstants
            // Make sure ModuleConstants.kDriveKV, kDriveKS, kDriveKA are set in Configs or Constants
            driveFeedforward = new SimpleMotorFeedforward(
                ModuleConstants.kDriveKS,
                ModuleConstants.kDriveKV,
                ModuleConstants.kDriveKA
            );

        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to initialize swerve drive");
        }

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        AHRS navx = (AHRS)swerveDrive.getGyro().getIMU();

        RobotConfig config;
            try{
            config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                e.printStackTrace();
                throw new RuntimeException("Failed to load RobotConfig");

            }

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
    }
    //hello
    /**
     * Drive the robot in teleop or autonomous
     * @param translation Translation vector in meters/sec
     * @param rotation Rotation in radians/sec
     * @param fieldRelative True if movement is relative to field
     * @param openLoop True for open-loop control (no PID)
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
        // Optional: calculate feedforward voltage for translation
        double targetSpeed = translation.getNorm(); // meters/sec
        double targetAccel = 0; // could be computed from trajectory
        double voltage = driveFeedforward.calculate(targetSpeed, targetAccel);

        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        swerveDrive.drive(speeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public void speedIncrease() {   
        if(DriveConstants.kMaxSpeedMetersPerSecond < 6.0)
            DriveConstants.kMaxSpeedMetersPerSecond += 1.0; 
        System.out.println("Max Speed: " + DriveConstants.kMaxSpeedMetersPerSecond);       
    }

    public void speedDecrease() {
        if(DriveConstants.kMaxSpeedMetersPerSecond > 3.0)
            DriveConstants.kMaxSpeedMetersPerSecond -= 1.0;
        System.out.println("Max Speed: " + DriveConstants.kMaxSpeedMetersPerSecond);       
    }

    /**
     * Optional: provide access to feedforward for autonomous or custom commands
     */
    public SimpleMotorFeedforward getDriveFeedforward() {
        return driveFeedforward;
    }
}