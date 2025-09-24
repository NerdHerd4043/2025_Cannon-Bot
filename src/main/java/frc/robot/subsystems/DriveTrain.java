// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(DriveConstants.frontLeftMotorID);
  private final WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(DriveConstants.frontRightMotorID);
  private final WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(DriveConstants.backLeftMotorID);
  private final WPI_TalonSRX backRightMotor = new WPI_TalonSRX(DriveConstants.backRightMotorID);

  private final Solenoid shifter = new Solenoid(RobotConstants.PCMID, PneumaticsModuleType.REVPH,
      DriveConstants.shifterID);

  public static boolean shiftVar = true;

  private final DifferentialDrive diffDrive = new DifferentialDrive(frontLeftMotor, frontRightMotor);;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {

    frontLeftMotor.configFactoryDefault();
    frontRightMotor.configFactoryDefault();
    backLeftMotor.configFactoryDefault();
    backRightMotor.configFactoryDefault();

    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    backLeftMotor.follow(frontLeftMotor);
    backRightMotor.follow(frontRightMotor);

    shift();
  }

  public void drive(double speed, double rot) {
    diffDrive.arcadeDrive(speed, rot, true);
  }

  public void shift() {
    boolean a;
    if (canShift()) {
      if (shiftVar) {
        a = !DriveConstants.lowGear;
        shiftVar = !shiftVar;
      } else {
        a = DriveConstants.lowGear;
        shiftVar = !shiftVar;
      }
      shifter.set(a);
    }
  }

  // cooldown

  long lastExecutionTime = 0;
  final long cooldownTime = 2000; // 2 seconds in milliseconds

  private boolean canShift() {
    // now reset current time
    long currentTime = System.currentTimeMillis();
    if (currentTime >= lastExecutionTime + cooldownTime) {
      // Execute the function
      System.out.println("Gear Shifted at " + currentTime + "ms.");
      lastExecutionTime = currentTime; // Update the last execution time
      // move it by setting bool to true
      return true;
    } else {
      // Cooldown is still active
      long timeLeft = (lastExecutionTime + cooldownTime) - currentTime;
      System.out.println("Gear on cooldown. Wait " + timeLeft + " ms.");
      return false;
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

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
