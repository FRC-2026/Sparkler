package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.LAUNCHING_LAUNCHER_VOLTAGE;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {

    private final SparkFlex launchingLaunch;

    public LauncherSubsystem()
    {
        launchingLaunch = new SparkFlex(LAUNCHER_MOTOR_ID, MotorType.kBrushless);

        // Config launchingLaunch motor
        SparkFlexConfig launchConfig = new SparkFlexConfig();
        launchConfig.inverted(true); // invert if needed
        launchConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        launchingLaunch.configure(launchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
    
    // A method to stop the rollers
    public void stopLaunch() {
    launchingLaunch.stopMotor();
    }

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
