// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.0);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private String selectedAuto;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    autoChooser.setDefaultOption("Move Backward", "moveBackward");
    autoChooser.addOption("Trough Coral", "troughCoral");
    autoChooser.addOption("Test Auto", "testAuto");
    SmartDashboard.putData("Auto Choices", autoChooser);
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {//-x forward +y right

  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);

    if(m_controller.getStartButton()) {
      m_swerve.resetGyro();
    }
  }

  @Override
  public void testPeriodic() {
      
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
