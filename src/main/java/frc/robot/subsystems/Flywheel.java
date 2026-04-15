  package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.SPIN_UP_MOTOR_ID;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 


  public class Flywheel extends SubsystemBase {

      private final SparkFlex spinUpFeeder;

      private final SparkClosedLoopController pid;
      private final RelativeEncoder encoder;

      private double targetRPM = -3000; // default target RPM
      private double kP = SmartDashboard.getNumber("kP", 0.0012);
      private double kI = SmartDashboard.getNumber("kI", 0.00001);
      private double kD = SmartDashboard.getNumber("kD", 0.00001);


      @Override
      public void periodic() {

      SmartDashboard.putNumber("FlywheelRPM", getRPM());
      Logger.recordOutput("Flywheel/CurrentRPM", getRPM());
      Logger.recordOutput("Flywheel/TargetRPM", targetRPM);


      }

      public Flywheel() {
          // Initialize motors
          spinUpFeeder = new SparkFlex(SPIN_UP_MOTOR_ID, MotorType.kBrushless);

          // Encoder on spinUpFeeder (use integrated encoder)
          encoder = spinUpFeeder.getEncoder();

          // PID controller on spinUpFeeder
          pid = spinUpFeeder.getClosedLoopController();

          // Dashboard default values
          SmartDashboard.putNumber("Target RPM", targetRPM);
          SmartDashboard.putNumber("kP", kP);
          SmartDashboard.putNumber("kI", kI);
          SmartDashboard.putNumber("kD", kD);

          // Config spinUpFeeder motor with PID
          SparkFlexConfig feederConfig = new SparkFlexConfig();
          feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);

          // Closed-loop PID + feedforward
          feederConfig.closedLoop
              .pid(kP, kI, kD)
              .feedForward.kS(0.2).kV(0.0025).kA(0.025); // shoul

          spinUpFeeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


          // Optional: invert spinUpFeeder if it spins wrong way
          // spinUpFeeder.setInverted(true);
      }

      

      /** Main control loop for flywheel */
      public void runFlywheel() {
          double currentRPM = encoder.getVelocity();

          // Directly use PID for velocity control
          pid.setSetpoint(targetRPM, ControlType.kVelocity);

          // Dashboard update
          SmartDashboard.putNumber("Flywheel RPM", currentRPM);
          SmartDashboard.putNumber("Target RPM", targetRPM);
          System.out.println(getRPM());
      }

      /** Run flywheel to a specific RPM */
      public void runFlywheel(double rpm) {
          targetRPM = rpm;
          runFlywheel();
      }

      /** Stop all motors */
      public void stopFlywheel() {
          spinUpFeeder.set(0);
      }

      // A method to stop the rollers
 

      public void stop()
      {
        spinUpFeeder.stopMotor();

      }

      /** Get current RPM */
      public double getRPM() {
          return encoder.getVelocity();
      }

      /** Set target RPM manually */
      public void setTargetRPM(double rpm) {
          targetRPM = rpm;
          SmartDashboard.putNumber("Target RPM", targetRPM);
      }

      /** Increase speed safely */
      public void speedIncrease(double increment, double maxRPM) {
          targetRPM -= increment;
          if (targetRPM < maxRPM) targetRPM = maxRPM;
          System.out.println("Flywheel: " + targetRPM);
      }

      /** Decrease speed safely */
      public void speedDecrease(double increment, double minRPM) {
          targetRPM += increment;
          if (targetRPM > minRPM) targetRPM = minRPM;
          System.out.println("Flywheel: " + targetRPM);
      }

      /** Command to run flywheel indefinitely */
      public Command spinUpCommand() {
          return this.run(this::runFlywheel);
      }

      /** Command that stops when RPM is close to target */
      // public Command spinUpToTargetCommand() {
      //     return this.run(this::runFlywheel)
      //     .until(() -> getRPM() >= targetRPM * 0.98);
      // }

      // A method to set the rollers to values for launching.







  }






