package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.studica.frc.AHRS;


public class DriveSubsystem extends SubsystemBase {
    // creates SwerveModules
    public final static MAXSwerveModule m_frontLeft = new MAXSwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.KFrontLeftTurningCancoderId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);

    public final static MAXSwerveModule m_frontRight = new MAXSwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.KFrontRightTurningCancoderId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);

    public final static MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.KRearLeftTurningCancoderId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);

    public final static MAXSwerveModule m_rearRight = new MAXSwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.KRearRightTurningCancoderId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    // NavX2 gyro (doesnt work 2026?)
    public static AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // gyro for field oriented swerves 
            
    // odometry class for tracking robot pose
    public static SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

        public DriveSubsystem() {
            // new path planner code in here 
        }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        };
    
        return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStates);
    }

    @Override
    public void periodic() {
        // Constantly updates pose for odometry
        m_odometry.update(
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            });
    }

    // Returns pose of robot 
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the given pose 
     * 
     * @param pose         desired pose of the robot
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            Rotation2d.fromDegrees(-m_gyro.getAngle()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        speed of the robot in the x direction (forward).
     * @param ySpeed        speed of the robot in the y direction (sideways).
     * @param rot           angular rate of the robot.
     * @param fieldRelative whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    Rotation2d.fromDegrees(-m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(   
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates the desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** 
     * Resets the drive encoders to currently read a position of 0. 
     */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** 
     * Zeroes the heading of the robot. 
     */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
    
    /**
     * Increases voltage of motors (speed) for drivetrain 
     */
    public void speedIncrease() {   
        if(DriveConstants.kMaxSpeedMetersPerSecond < 6.0)
        DriveConstants.kMaxSpeedMetersPerSecond += 1.0; 
        System.out.println("Max Speed: " + DriveConstants.kMaxSpeedMetersPerSecond);       
    }

    /**
     * Decreases voltage of motors (speed) for drivetrain 
     */
    public void speedDecrease() {
        if(DriveConstants.kMaxSpeedMetersPerSecond > 3.0)
        DriveConstants.kMaxSpeedMetersPerSecond -= 1.0;
        System.out.println("Max Speed: " + DriveConstants.kMaxSpeedMetersPerSecond);       
    }
}