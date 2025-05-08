// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModule {
  private static final double kWheelRadius = 0.0375;
  private static final double kDriveRatio = 5.14;
  private static final double kTurnRatio = 1.0;
  private static final int kEncoderResolution = 4096;
  private final double kEncoderOffset;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final SparkMax m_driveMotor;
  private final SparkMaxConfig m_driveConfig;
  private final TalonSRX m_turningMotor;

  private final RelativeEncoder m_driveEncoder;

  private final Translation2d position;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  public final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.5,
          0,
          0.00,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.05);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.05, 0.01);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel, Translation2d _position, double encoderOffset) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);

    m_driveConfig = new SparkMaxConfig();
    m_driveConfig.inverted(false);
    m_driveMotor.configure(m_driveConfig, null, null);
    m_turningMotor = new TalonSRX(turningMotorChannel);
    m_turningMotor.clearStickyFaults();

    position = _position;

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    System.out.println("Module" + turningMotorChannel + ": " + getTurningEncoderPosition());
    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    
    kEncoderOffset = m_turningMotor.getSelectedSensorPosition();

    System.out.println("After Adjustment");
    System.out.println("Module" + turningMotorChannel + ": " + getTurningEncoderPosition());
    System.out.println("Module" + turningMotorChannel + " Offset: " + kEncoderOffset);
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocity(), new Rotation2d(getTurningEncoderPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderPosition(), new Rotation2d(getTurningEncoderPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getTurningEncoderPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.speedMetersPerSecond *= desiredState.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(getDriveEncoderVelocity(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput =
        m_turningPIDController.calculate(getTurningEncoderPosition(), desiredState.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    //if(position.getX() >= 0.0) {turnOutput = -1.0*turnOutput;}
    
    m_driveMotor.setVoltage(driveOutput*5.0 + driveFeedforward);
    m_turningMotor.set(ControlMode.PercentOutput,turnOutput + (turnFeedforward)/12.0);

    //System.out.println("PID Setpoint: " + m_turningPIDController.getSetpoint().position/(2.0*Math.PI));
  }
  

  public double getTurningEncoderPosition() {
    //System.out.println(m_turningMotor.getSelectedSensorPosition() + "ticks " + m_turningMotor.getSelectedSensorPosition()*(360.0/kTurnRatio/kEncoderResolution));
    return (m_turningMotor.getSelectedSensorPosition()-kEncoderOffset)*(2.0*Math.PI/kTurnRatio/kEncoderResolution);
  }

  public double getDriveEncoderPosition() {
    return m_driveEncoder.getPosition() * (2.0*Math.PI*kWheelRadius/kDriveRatio);
  }

  public double getDriveEncoderVelocity() {
    return m_driveEncoder.getVelocity() * (2.0*Math.PI*kWheelRadius/kDriveRatio)/60.0;
  }

  public void zero() {
    System.out.println("Zeroed Module");
    m_turningMotor.setSelectedSensorPosition(0.0);
  }
}

