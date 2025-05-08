// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 12.0; // 3 meters per second
  public static final double kMaxAngularSpeed = 6.0*Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.33, 0.33);
  private final Translation2d m_frontRightLocation = new Translation2d(0.33, -0.33);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.33, 0.33);
  private final Translation2d m_backRightLocation = new Translation2d(-0.33, -0.33);

  public final SwerveModule m_frontLeft = new SwerveModule(2, 8, m_frontLeftLocation, 0.0); //1565
  public final SwerveModule m_frontRight = new SwerveModule(4, 7, m_frontRightLocation, 0.0);//-9820.0 mod 4096
  public final SwerveModule m_backLeft = new SwerveModule(5, 9, m_backLeftLocation, 0.0); //1750
  public final SwerveModule m_backRight = new SwerveModule(3, 6, m_backRightLocation, 0.0); //2646

  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          gyroRotation(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
    m_odometry.resetPose(new Pose2d(0,0,new Rotation2d(0)));
    System.out.println("Gyro:");
    System.out.println(m_gyro.toString());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, gyroRotation())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(),//gyroRotation(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  private Rotation2d gyroRotation() {
    return new Rotation2d(m_gyro.getAngle());
  }

  public void resetGyro() {
    m_gyro.reset();
  }
}
