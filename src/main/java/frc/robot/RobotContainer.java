// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;

@Logged
public class RobotContainer {
  // Creates our controller
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system
  @NotLogged
  private final CommandXboxController driveStick = new CommandXboxController(0);

  private final DriveTrain drivetrain = new DriveTrain();

  // Creates our subsystems

  // create new demo sendablechooser
  private SendableChooser<Command> demoChooser;

  // Algae commands are mostly untested
  Command genericCommand = Commands.parallel(
  // fortnite.playFortnite(),
  );

  // Tells the robot to drive by default.
  public RobotContainer() {

    this.configureBindings();

    drivetrain.setDefaultCommand(
        new RunCommand(
            () -> drivetrain.drive(
                driveStick.getLeftY(),
                driveStick.getRightX()),
            drivetrain));

  }

  private double[] getScaledXY() {
    // Array for storing the x/y inputs from the controller
    double[] xy = new double[2];

    // Assigning inputs to array locations. X and Y are switched because the
    // controller is funky.

    return xy;
  }

  public void resetGyro() {
  }

  private void configureBindings() {

    /* Intake/Output buttons */
    // Intake

    // make sure off cooldown before doing again

    driveStick.leftBumper().onTrue(
        Commands.parallel(
            /// commands here
            new InstantCommand(drivetrain::shiftUp, drivetrain)

        ));
    // Output
    driveStick.rightBumper().onTrue(
        Commands.parallel(
            /// commands here
            new InstantCommand(drivetrain::shiftDown, drivetrain)

        ));
    // DRIVESTICK BUTTONS
    driveStick.b().whileTrue(
        Commands.parallel(
        // commands here
        ));

    // L2
    driveStick.a().onTrue(Commands.parallel(

    // commansd here

    ));
    // L3
    driveStick.x().onTrue(
        Commands.parallel(
        // commands here

        ));
    // L4
    driveStick.y().onTrue(
        Commands.parallel(
        // commands here

        ));

    /* Reset gyro button */

    Trigger leftTriggerLow = driveStick.leftTrigger(0.1);
    Trigger leftTriggerHigh = driveStick.leftTrigger(0.9);

    leftTriggerHigh
        .whileTrue(Commands.parallel(
        // write commands here

        ));

    driveStick.rightTrigger()
        .onTrue(Commands.sequence(
        // write commands here
        ));

    var fullRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kBothRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kBothRumble, 0));

    var leftRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kLeftRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kLeftRumble, 0));

    var leftAlignCommand = Commands.parallel(
        leftRumbleCommand.withTimeout(0.5));

    driveStick.leftStick().toggleOnTrue(leftAlignCommand);
    driveStick.povLeft().toggleOnTrue(leftAlignCommand);

    driveStick.start().onTrue(Commands.parallel(

        // commands here
        fullRumbleCommand.withTimeout(0.5)));

    var rightRumbleCommand = Commands.startEnd(
        () -> driveStick.setRumble(RumbleType.kRightRumble, 0.5),
        () -> driveStick.setRumble(RumbleType.kRightRumble, 0));

    var rightAlignCommand = Commands.parallel(
        rightRumbleCommand.withTimeout(0.5));

    driveStick.rightStick().toggleOnTrue(rightAlignCommand);
    driveStick.povRight().toggleOnTrue(rightAlignCommand);
  }

}
