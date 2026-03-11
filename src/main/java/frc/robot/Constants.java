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
  public static final double maxspeed = Units.feetToMeters(5.5);
  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LAUNCHER_MOTOR_ID = 13;
    public static final int SPIN_UP_MOTOR_ID = 14;
    public static final int INTAKE_INTAKE_MOTOR_ID = 15;
    public static final int INTAKE_ARM_MOTOR_ID = 16;
    
    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_ARM_VOLTAGE = 1;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static double SPIN_UP_FEEDER_VOLTAGE = 6;
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
    public static final double kTrackWidth = Units.inchesToMeters(27.5);
    public static final double kWheelBase = Units.inchesToMeters(27.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // front left 
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // front right
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), // back left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); // back right


    // SPARK MAX CAN IDs - Must change 
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 2;
    
    public static final int KFrontLeftTurningCancoderId = 9;
    public static final int KFrontRightTurningCancoderId = 11;
    public static final int KRearLeftTurningCancoderId = 10;
    public static final int KRearRightTurningCancoderId = 12;

  }
    
  public static final class ModuleConstants { 
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60.0; 
    public static final double kWheelDiameterMeters = 0.1016; // 4" 
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // MK4i L3 gear ratio 
    public static final double kDrivingMotorReduction = 6.12; 
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction; 
    public static double kDriveKV; // volts per m/s
    public static double kDriveKS; // static voltage
    public static double kDriveKA; // acceleration voltage
  }

  public static final class OIConstants {
    // may change - controller's usb port
    public static final int kDriverControllerPort = 0;
    // avoids inputting small joystick movements, adjustable
    public static final double kDriveDeadband = 0.05;
  }

  public static final class FeedforwardConstants {
    public static final double kS = 0.2;  // Static voltage
    public static final double kV = 12 / ((NeoMotorConstants.kFreeSpeedRpm / 60.0) * 2 * Math.PI);
    public static final double kA = 0.05; // Tuning required
  }

  // RPM = revolutions per minute, unit of rotational speed
  // free speed AKA max speed it can achieve not under load ex. lifting and arm weight 
  public static final class NeoMotorConstants { 
    public static final double kFreeSpeedRpm = 5820;       // Free speed (no load)
    public static final double kStallTorqueNm = 2.6;       // Stall torque in Newton-meters
    public static final double kStallCurrentA = 105;       // Stall current in Amps
    public static final double kFreeCurrentA = 1.5;        // Free current in Amps
    public static final double kMotorResistance = 12 / kStallCurrentA; // Ohms  }
  }

}