// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CannonConstants;

public class CannonManipulator extends SubsystemBase {
  private final SparkMax manipulatorMotor = new SparkMax(CannonConstants.manipulatorMotorID, MotorType.kBrushless);

  private RelativeEncoder encoder = manipulatorMotor.getEncoder();

  /** Creates a new CannonManipulator. */
  public CannonManipulator() {
    SparkMaxConfig manipulatorMotorConfig = new SparkMaxConfig();
    manipulatorMotorConfig.idleMode(IdleMode.kBrake);

    manipulatorMotor.configure(manipulatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // private boolean withinMaxBounds() {
  // // TODO: write hard stop logic

  // return !(encoder.getPosition() >= -3);
  // }

  // private boolean withinMinBounds() {
  // // TODO: write hard stop logic

  // return !(encoder.getPosition() <= -1);
  // }

  public Command up() {
    return this.run(() -> {
      // if (withinMaxBounds()) {
      manipulatorMotor.set(0.05);
      // }
    }).finallyDo(() -> {
      manipulatorMotor.stopMotor();
    });

  }

  public Command down() {
    return this.run(() -> {
      // if (withinMinBounds()) {
      manipulatorMotor.set(-0.05);
      // }
    }).finallyDo(() -> {
      manipulatorMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {

  }
}
