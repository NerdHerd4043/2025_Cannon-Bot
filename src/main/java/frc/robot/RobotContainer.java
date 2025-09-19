// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Cannon;
import frc.robot.subsystems.CannonManipulator;

@Logged
public class RobotContainer {
        // private final CannonManipulator cannonManipulator = new CannonManipulator();
        // Creates our controller
        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#joystick-and-controller-coordinate-system
        @NotLogged
        private final CommandXboxController driveStick = new CommandXboxController(0);

        private final DriveTrain drivetrain = new DriveTrain();
        // private Cannon cannon = new Cannon();

        // Creates our subsystems

        // create new demo sendablechooser

        // Tells the robot to drive by default.
        public RobotContainer() {

                // this.configureBindings();

                drivetrain.setDefaultCommand(
                                new RunCommand(() -> drivetrain.drive(driveStick.getLeftY(), driveStick.getRightX()),
                                                drivetrain));

        }

        public void resetGyro() {
        }

        public void rumbleController() {

        }

        // private void configureBindings() {

        // driveStick.leftBumper().onTrue(
        // drivetrain.runOnce(drivetrain::shiftUp));
        // // Output
        // driveStick.rightBumper().onTrue(
        // /// commands here
        // drivetrain.runOnce(drivetrain::shiftDown));

        // driveStick.rightTrigger().onTrue(
        // cannon.runOnce(cannon::shoot));

        // driveStick.rightTrigger().onFalse(
        // cannon.runOnce(cannon::resetTrigger));

        // var fullRumbleCommand = Commands.startEnd(
        // () -> driveStick.setRumble(RumbleType.kBothRumble, 0.5),
        // () -> driveStick.setRumble(RumbleType.kBothRumble, 0));

        // driveStick.x().whileTrue(cannonManipulator.up());
        // driveStick.b().whileTrue(cannonManipulator.down());
        // }
}
