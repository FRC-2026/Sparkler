package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ModuleConstants;

public class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

 static {
    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
    double steeringReduction = 150.0 / 7.0; // MK4i L3
    double turningFactor = (2 * Math.PI) / steeringReduction;

    // --- DRIVING CONFIG ---
    drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);

    drivingConfig.encoder
        .positionConversionFactor(drivingFactor)
        .velocityConversionFactor(drivingFactor / 60.0);

    drivingConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.08, 0, 0)
        .outputRange(-1, 1);

    // --- Feedforward calculation ---
    // kV = volts per m/s, kS = static friction voltage, kA = volts per m/s^2
    double kV = 12 / ModuleConstants.kDriveWheelFreeSpeedRps; // volts per m/s
    double kS = 0.2; // small voltage to overcome friction
    double kA = 0.05; // acceleration voltage, adjust after testing

    // store feedforward somewhere accessible, e.g., ModuleConstants or Configs
    // Example:
    ModuleConstants.kDriveKV = kV;
    ModuleConstants.kDriveKS = kS;
    ModuleConstants.kDriveKA = kA;

    // --- TURNING CONFIG ---
    turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(20);

    turningConfig.encoder
        .positionConversionFactor(turningFactor)
        .velocityConversionFactor(turningFactor / 60.0);

    turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(5, 0, 0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI);
}
        public Command autoBalanceCommand() {
            throw new UnsupportedOperationException("Unimplemented method 'autoBalanceCommand'");
        }
    }
}