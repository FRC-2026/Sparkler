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
            // use module constants to calculate conversion factors and feed forward gain
            // ex. driv f
            //factor converts motor rotations to metres
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
            double steeringReduction = 150.0 / 7.0; // MK4i L2
            double turningFactor = (2 * Math.PI) / steeringReduction;

            drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // encoder ticks to metres
                .velocityConversionFactor(drivingFactor / 60.0); // encoder velocity to m/s
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // may need to change, higher p reacts faster but can cause unwanted back and forth movement
                // integral is drift correction derivitave is reduces sudden changes
                .pid(0.08, 0, 0)//okok
                .outputRange(-1, 1);

            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);

            turningConfig.encoder
                .positionConversionFactor(turningFactor)   // motor rotations → radians
                .velocityConversionFactor(turningFactor / 60.0); // rad/sec
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)  // <-- IMPORTANT
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