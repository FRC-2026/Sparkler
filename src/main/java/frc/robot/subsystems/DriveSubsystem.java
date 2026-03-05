package frc.robot.subsystems;

import java.io.File;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

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

    }

    /**
     * Drive the robot in teleop or autonomous
     * @param translation Translation vector in meters/sec
     * @param rotation Rotation in radians/sec
     * @param fieldRelative True if movement is relative to field
     * @param openLoop True for open-loop control (no PID)
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
        // Optional: calculate feedforward voltage for translation
        // double targetSpeed = translation.getNorm(); // meters/sec
        // double targetAccel = 0; // could be computed from trajectory
        // double voltage = driveFeedforward.calculate(targetSpeed, targetAccel);

        swerveDrive.drive(translation, rotation, true, openLoop);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
        System.out.println("Allan sucks");
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