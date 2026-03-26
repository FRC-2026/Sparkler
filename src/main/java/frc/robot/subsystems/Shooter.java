// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class Shooter extends SubsystemBase {
  private final SparkFlex launchingLaunch;
  private final SparkFlex spinUpFeeder;

  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public Shooter() {
    // create brushless motors for each of the motors on the launcher mechanism
    launchingLaunch = new SparkFlex(LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    spinUpFeeder = new SparkFlex(SPIN_UP_MOTOR_ID, MotorType.kBrushless);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    SparkFlexConfig launchingLaunchConfig = new SparkFlexConfig();
    launchingLaunchConfig.inverted(true);
    launchingLaunchConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    launchingLaunch.configure(launchingLaunchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig spinUpFeederConfig = new SparkFlexConfig();
    spinUpFeederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    spinUpFeeder.configure(spinUpFeederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }


  // A method to set the rollers to values for launching.
  public void launch() {
    launchingLaunch
      .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stopLaunch() {
    launchingLaunch.stopMotor();
  }

  public void stopSpinUP()
  {
    spinUpFeeder.stopMotor();
  }

  public void stop()
  {
    spinUpFeeder.stopMotor();
    launchingLaunch.stopMotor();


  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  // constants used if nothing inside smart dashboard
  public void spinUp() {
    spinUpFeeder
    .setVoltage(-1 * SmartDashboard.getNumber("Spin up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));

    // launchingLaunch
    // .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  public void launchReverse()
  {
    launchingLaunch
      .setVoltage(-1*SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));

  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public Command reverseLaunchCommand(){
    return this.run(()-> launchReverse());
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Increases voltage of motors for shooter to shoot balls higher and faster 
  public void speedIncrease()
  {
    if (SPIN_UP_FEEDER_VOLTAGE < 13.0)
    {
      LAUNCHING_LAUNCHER_VOLTAGE += 1.0;
      SPIN_UP_FEEDER_VOLTAGE += 1.0;
      System.out.println(LAUNCHING_LAUNCHER_VOLTAGE);
      System.out.println(SPIN_UP_FEEDER_VOLTAGE);
    }
  }
  
  // Decreases voltage of motors for shooter to shoot balls lower and slower 
  public void speedDecrease()
  {
    if (SPIN_UP_FEEDER_VOLTAGE > 1.0)
    {
      LAUNCHING_LAUNCHER_VOLTAGE -= 1.0;
      SPIN_UP_FEEDER_VOLTAGE -= 1.0;
      System.out.println(LAUNCHING_LAUNCHER_VOLTAGE);
      System.out.println(SPIN_UP_FEEDER_VOLTAGE);
    }
  }

}
