// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 13;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 14;

    
    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

  }

  public final class DriveConstants {
  
    public static double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; 

    // chassis width and length
    public static final double kTrackWidth = Units.inchesToMeters(28.5);
    public static final double kWheelBase = Units.inchesToMeters(30.4);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front right
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // back right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // front left 
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back left

    // encoder offsets as wheels starting positions, 0, won't be exact with the chassis frame
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; // -90 deg
    public static final double kFrontRightChassisAngularOffset = 0; 
    public static final double kBackLeftChassisAngularOffset = Math.PI; // 180 deg
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; // 90 deg

    // SPARK MAX CAN IDs - Must change 
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 2;
    
    public static final int KFrontLeftTurningCancoderId = 9;
    public static final int KFrontRightTurningCancoderId = 10;
    public static final int KRearLeftTurningCancoderId = 12;
    public static final int KRearRightTurningCancoderId = 13;
    


    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // May change - count teeth
    public static final int kDrivingMotorPinionTeeth = 14;

    // May change? (need to change)
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // May change - 45 teeth for wheel's bevel gear 22 for first-stage spur gear, 15 for bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    // may change - controller's usb port
    public static final int kDriverControllerPort = 0;
    // avoids inputting small joystick movements, adjustable
    public static final double kDriveDeadband = 0.05;
  }

  // RPM = revolutions per minute, unit of rotational speed
  // free speed AKA max speed it can achieve not under load ex. lifting and arm weight 
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
