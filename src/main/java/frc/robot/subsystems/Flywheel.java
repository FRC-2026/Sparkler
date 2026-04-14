  package frc.robot.subsystems;

  import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 


  public class Flywheel extends SubsystemBase {

      private final SparkFlex spinUpFeeder;
      private final SparkFlex launchingLaunch;

      private final SparkClosedLoopController pid;
      private final RelativeEncoder encoder;

      private double targetRPM = -2000; // default target RPM
      private double kP = SmartDashboard.getNumber("kP",0.000000001);
      private double kI = SmartDashboard.getNumber("kI", 0);
      private double kD = SmartDashboard.getNumber("kD", 0);


      @Override
      public void periodic() {

      SmartDashboard.putNumber("FlywheelRPM", getRPM());
      Logger.recordOutput("Flywheel/CurrentRPM", getRPM());
      Logger.recordOutput("Flywheel/TargetRPM", targetRPM);


      }

      public Flywheel() {
          // Initialize motors
          spinUpFeeder = new SparkFlex(SPIN_UP_MOTOR_ID, MotorType.kBrushless);
          launchingLaunch = new SparkFlex(LAUNCHER_MOTOR_ID, MotorType.kBrushless);

          // Encoder on spinUpFeeder (use integrated encoder)
          encoder = spinUpFeeder.getEncoder();

          // PID controller on spinUpFeeder
          pid = spinUpFeeder.getClosedLoopController();

          // Dashboard default values
          SmartDashboard.putNumber("Target RPM", targetRPM);
          SmartDashboard.putNumber("kP", kP);
          SmartDashboard.putNumber("kI", kI);
          SmartDashboard.putNumber("kD", kD);


          // Config launchingLaunch motor
          SparkFlexConfig launchConfig = new SparkFlexConfig();
          launchConfig.inverted(true); // invert if needed
          launchConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
          launchingLaunch.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

          // Config spinUpFeeder motor with PID
          SparkFlexConfig feederConfig = new SparkFlexConfig();
          feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);

          // Closed-loop PID + feedforward
          feederConfig.closedLoop
            .pid(0.0004, 0.0000000001, 0.0)
            .feedForward
                .kS(0.2)
                .kV(0.0020)  // tune this
                .kA(0.0);

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
      public void stopLaunch() {
        launchingLaunch.stopMotor();
      }

      public void stop()
      {
        spinUpFeeder.stopMotor();
        launchingLaunch.stopMotor();
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
      public void launch() {
        launchingLaunch
          .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
      }

      public void launchReverse()
      {
        launchingLaunch
          .setVoltage(-1 * SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
      }

      // A command factory to turn the launch method into a command that requires this
      // subsystem
      public Command launchCommand() {
        return this.run(() -> launch());
      }

      public Command reverseLaunchCommand(){
        return this.run(()-> launchReverse());
      }
      

   



  }






