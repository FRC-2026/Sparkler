package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Configs;
import com.ctre.phoenix.sensors.*;
import com.ctre.phoenix6.hardware.CANcoder;


public class MAXSwerveModule {
  private final CANcoder m_cancoder;        // absolute encoder

  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  public final RelativeEncoder m_drivingEncoder;
  public final RelativeEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;
  
  private double absoluteRotations;
  private double absoluteAngleRad;



  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());



  
  // docs use NEOs, SPARKS MAX, and through Magnetic CANcoder encoder.
  public MAXSwerveModule(int drivingCANId, int cancoderCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);
    m_cancoder = new CANcoder(cancoderCANId);//match the cancoderId


    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getEncoder();
    absoluteRotations = m_cancoder.getPosition().getValueAsDouble(); // gives the rotations
    absoluteAngleRad = absoluteRotations*2*Math.PI; //gives the radian, so it can be used
    //Basically so that the module is robot orientated rather than module or
    absoluteAngleRad -= chassisAngularOffset;     // apply chassis offset so that the aboslute encoder is correct for every wheel, as the foward may not be the same for every wheel depending on how the chassis is facing, so this subtracts it so that they know that forsay 0 rad is fowrads. 

    m_turningEncoder.setPosition(absoluteAngleRad);//updates the turning encoder to have the aboslute angle.

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();//configuration for pid
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    absoluteAngleRad -= m_chassisAngularOffset;

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(absoluteAngleRad); //this only happens at start up so once per module. 
    m_drivingEncoder.setPosition(0); //starting 0 at that posistion doesn't matter where lol
  }

  // returns the current state of the module.
  public SwerveModuleState getState() {
    // apply chassis angular offset to the encoder position to get the position
    // relative to the chassis
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // returns the current position of the module.
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // desired state with speed and anglea
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // optimize the reference state to avoid spinning further than 90 degrees
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // command driving and turning SPARKS towards their respective setpoints
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  // zeroes all the SwerveModule encoders
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}