// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

public class Intake extends SubsystemBase {
    private final SparkMax intakeRoller;
    private final SparkMax indexArm;

    @SuppressWarnings("removal")
    public Intake() {
        intakeRoller = new SparkMax(INTAKE_INTAKE_MOTOR_ID, MotorType.kBrushless);
        indexArm = new SparkMax(INTAKE_ARM_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig intakeRollerConfig = new SparkMaxConfig();
        intakeRollerConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        intakeRoller.configure(intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig indexArmConfig = new SparkMaxConfig();
        indexArmConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        indexArm.configure(indexArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_INTAKE_VOLTAGE);
        SmartDashboard.putNumber("Intaking intake roller value", INTAKING_ARM_VOLTAGE);
    }
    

    // A method to stop the rollers
    public void stopIntake() {
      intakeRoller.stopMotor();
    }

    public void stopIntakeArm() {
      indexArm.stopMotor();
    }


    // A method to set the rollers to values for intaking
    public void intake() {
        intakeRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_INTAKE_VOLTAGE));
    }

    // A method to move intake arm


    public void indexDown() {
        indexArm.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_ARM_VOLTAGE));
    }

    // A method to move intake arm
    public void reverseIndexArm() {
        indexArm.setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_ARM_VOLTAGE));
    }

    // A method to set the rollers to values for ejecting fuel out the intake. Uses
    // the same values as intaking, but in the opposite direction.
    public void eject() {
        intakeRoller
            .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_INTAKE_VOLTAGE));
    }


    
    // A method to stop the rollers
    public void stopRoller() {
        intakeRoller.stopMotor();
    }

    public void stopIndexArm() {
        indexArm.stopMotor();
    }

    public Command intakeCommand() {
    return this.run(() -> intake());
  }

    public Command indexArmCommand() {
    return this.run(() -> indexDown());
  }

    public Command reverseIndexArmCommand() {
    return this.run(() -> reverseIndexArm());
  }

    public Command ejectCommand() {
    return this.run(() -> eject());
  }
}
