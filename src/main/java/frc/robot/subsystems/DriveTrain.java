// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class DriveTrain extends SubsystemBase {
  private final SparkMax leftDriveMotor = new SparkMax(DriveConstants.leftMotorID, MotorType.kBrushless);
  private final SparkMax rightDriveMotor = new SparkMax(DriveConstants.rightMotorID, MotorType.kBrushless);

  Solenoid shifter = new Solenoid(RobotConstants.PCMID, PneumaticsModuleType.CTREPCM, DriveConstants.shifterID);

  DifferentialDrive diffDrive;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {

    SparkMaxConfig leftDriveMotorConfig = new SparkMaxConfig();
    SparkMaxConfig rightDriveMotorConfig = new SparkMaxConfig();

    leftDriveMotor.configure(leftDriveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightDriveMotor.configure(rightDriveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    diffDrive = new DifferentialDrive(rightDriveMotor, leftDriveMotor);

  }

  public void drive(Double speed, Double rot) {
    diffDrive.arcadeDrive(speed, rot, true);
  }

  public void stop() {
    drive(0., 0.);
  }

  public void shift(boolean a) {
    shifter.set(a);
  }

  // cooldown

  long lastExecutionTime = 0;
  final long cooldownTime = 2000; // 2 seconds in milliseconds
  boolean canShift = false;

  private void getGearboxCooldown() {

    // set bool false by default
    canShift = false;
    // now reset current time
    long currentTime = System.currentTimeMillis();
    if (currentTime >= lastExecutionTime + cooldownTime) {
      // Execute the function
      System.out.println("Gear Shifted at " + currentTime + "ms.");
      lastExecutionTime = currentTime; // Update the last execution time

      // move it by setting bool to true
      canShift = true;
    } else {
      // Cooldown is still active
      long timeLeft = (lastExecutionTime + cooldownTime) - currentTime;
      System.out.println("Gear on cooldown. Wait " + timeLeft + " ms.");
    }

  }

  public void shiftUp() {
    // check gear cooldown for gearbox; if off cooldown, set bool and run command
    getGearboxCooldown();
    if (canShift) {
      shift(!DriveConstants.lowGear);
    }
  }

  public void shiftDown() {
    // check gear cooldown for gearbox; if off cooldown, set bool and run command
    getGearboxCooldown();
    if (canShift) {
      shift(DriveConstants.lowGear);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
