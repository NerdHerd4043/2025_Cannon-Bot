// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CannonConstants;

public class Cannon extends SubsystemBase {

  private final Servo triggerServo = new Servo(CannonConstants.triggerPort);

  /** Creates a new CannonManipulator. */
  public Cannon() {
    // reset trigger on startup
    this.resetTrigger();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    triggerServo.set(CannonConstants.shootPosition);

  }

  public void resetTrigger() {
    triggerServo.set(CannonConstants.restPosition);

  }

}
