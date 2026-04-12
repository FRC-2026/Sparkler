// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // private CANcoder cancoder;
  // private CANcoder cancoder1;
  // private CANcoder cancoder2;
  // private CANcoder cancoder3;

  private final Field2d m_field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setPixelFormat(PixelFormat.kMJPEG);
    camera.setResolution(320, 240);
    camera.setExposureManual(40);
    camera.setWhiteBalanceAuto();
    camera.setFPS(30);

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    DataLogManager.start();//command to start the operation
    DriverStation.startDataLog(DataLogManager.getLog());//addidng all the datas

    Logger.start();//starts logging

    Logger.recordMetadata("ProjectName", "MyRobot");

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);

    // uncomment to track CANcoder positions and set new offsets if needed
    // cancoder = new CANcoder(9);
    // cancoder1 = new CANcoder(10);
    // cancoder2 = new CANcoder(11);
    // cancoder3 = new CANcoder(12);
    // double absolutePosition = cancoder.getAbsolutePosition().getValueAsDouble() * 360.0; // Convert to degrees
    // double absolutePosition1 = cancoder1.getAbsolutePosition().getValueAsDouble() * 360.0; // Convert to degrees
    // double absolutePosition2 = cancoder2.getAbsolutePosition().getValueAsDouble() * 360.0; // Convert to degrees
    // double absolutePosition3 = cancoder3.getAbsolutePosition().getValueAsDouble() * 360.0; // Convert to degrees

    // SmartDashboard.putNumber("CANcoder Absolute Position", absolutePosition); // front right 
    // SmartDashboard.putNumber("CANcoder Absolute Position1", absolutePosition1); // back right
    // SmartDashboard.putNumber("CANcoder Absolute Position2", absolutePosition2); // front left
    // SmartDashboard.putNumber("CANcoder Absolute Position3", absolutePosition3); // back left 

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("SwerveDrive");

        // builder.addDoubleProperty("Front Left Angle", () -> DriveSubsystem.m_frontLeft.m_turningEncoder.getPosition(), null);
        // builder.addDoubleProperty("Front Left Velocity", () -> DriveSubsystem.m_frontLeft.m_drivingEncoder.getVelocity(), null);

        // builder.addDoubleProperty("Front Right Angle", () -> DriveSubsystem.m_frontRight.m_turningEncoder.getPosition(), null);
        // builder.addDoubleProperty("Front Right Velocity", () -> DriveSubsystem.m_frontRight.m_drivingEncoder.getVelocity(), null);

        // builder.addDoubleProperty("Back Left Angle", () -> DriveSubsystem.m_rearLeft.m_turningEncoder.getPosition(), null);
        // builder.addDoubleProperty("Back Left Velocity", () -> DriveSubsystem.m_rearLeft.m_drivingEncoder.getVelocity(), null);

        // builder.addDoubleProperty("Back Right Angle", () -> DriveSubsystem.m_rearRight.m_turningEncoder.getPosition(), null);
        // builder.addDoubleProperty("Back Right Velocity", () -> DriveSubsystem.m_rearRight.m_drivingEncoder.getVelocity(), null);

        // builder.addDoubleProperty("Robot Angle", () -> DriveSubsystem.m_gyro.getAngle(), null);
      }
    });
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // // block in order for anything in the Command-based framework to work.
    // m_field.setRobotPose(DriveSubsystem.m_odometry.getPoseMeters());
    // SmartDashboard.putNumber("Max Speed", Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    // SmartDashboard.putNumber("Match Timer", DriverStation.getMatchTime());
    // SmartDashboard.putNumber("Gyro", DriveSubsystem.m_gyro.getAngle());
    // SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
    // SmartDashboard.putNumber("Shooter voltage from controller: ", Constants.FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE);
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
