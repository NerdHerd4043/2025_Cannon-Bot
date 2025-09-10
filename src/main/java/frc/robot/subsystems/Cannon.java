// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CannonConstants;
import frc.robot.Constants.DriveConstants;

public class Cannon extends SubsystemBase {

  private final Servo triggerServo = new Servo(CannonConstants.triggerPort);

  private final SparkMax manipulatorMotor = new SparkMax(5, MotorType.kBrushless);

  /** Creates a new CannonManipulator. */
  public Cannon() {
    SparkMaxConfig manipulatorMotorConfig = new SparkMaxConfig();

    manipulatorMotor.configure(manipulatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
