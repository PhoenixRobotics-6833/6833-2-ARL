// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends TimedRobot {

  // INITIALIZE VARIABLES

  final int SPEED = 1;
  // CONTROLLER SETUP
  XboxController primaryController = new XboxController(0);
  XboxController secondaryController = new XboxController(1);

  // MOTOR SETUP
  final TalonSRX motorFrontRight = new TalonSRX(0);
  final TalonSRX motorFrontLeft = new TalonSRX(1);
  final TalonSRX motorBackRight = new TalonSRX(2);
  final TalonSRX motorBackLeft = new TalonSRX(3);
  final TalonSRX motorArm = new TalonSRX(4);

  // LOGICAL VARIABLES
  boolean armRetract = false;

  void setMotors(double leftThrottle, double rightThrottle) {
    motorBackLeft.set(TalonSRXControlMode.PercentOutput, leftThrottle * SPEED);
    motorBackRight.set(TalonSRXControlMode.PercentOutput, -rightThrottle * SPEED);
    motorFrontLeft.set(TalonSRXControlMode.PercentOutput, leftThrottle * SPEED);
    motorFrontRight.set(TalonSRXControlMode.PercentOutput, -rightThrottle * SPEED);
  }

  @Override
  public void teleopPeriodic() {
    double throttle = primaryController.getRawAxis(3) - primaryController.getRawAxis(2);
    double steer = primaryController.getRawAxis(0);
    setMotors(throttle + steer, throttle - steer);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
