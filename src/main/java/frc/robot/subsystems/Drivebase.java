// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import frc.robot.Constants;

@Logged
public class Drivebase extends SubsystemBase {
  private static NetworkTable alignTable = NetworkTableInstance.getDefault()
      .getTable("alignment");
  private static DoublePublisher currentXPub = alignTable.getDoubleTopic("Robot Pose X").publish();

  @NotLogged
  private SwerveDriveOdometry odometry;

  private Field2d field = new Field2d();

  private BooleanEntry fieldOrientedEntry;

  // Creates Sendables on the dashboard that can be interacted with to affect the
  // robot without pushing new code. These ones end up being used for scaling the
  // robot's drive speed, choosing between field and robot oriented drive, and
  // choosing the reef positions to move to.
  private SendableChooser<Double> driveSpeedChooser = new SendableChooser<>();
  private SendableChooser<Boolean> fieldOriented = new SendableChooser<>();
  private SendableChooser<Double> leftReefTargetPose = new SendableChooser<>();
  private SendableChooser<Double> rightReefTargetPose = new SendableChooser<>();

  // The Subscriber "subscribes" to a piece of information, allowing the
  // information to be recieved and updated. Source:
  // https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html#subscribing-to-a-topic
  @NotLogged
  private final DoubleArraySubscriber R_limelightRobotPose;
  // This double array is used later to hold information we get from the
  // subscriber. Limelight documentation (as of now) doesn't use a subscriber, but
  // our subscriber is getting the same values that the `botpose_targetspace`
  // gets, and those values are best stored in a double array. Source:
  // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
  @NotLogged
  private double[] R_limelightRobotPoseArray = new double[6];

  //
  //
  /** Creates a new Drivebase. */
  public Drivebase() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("SmartDashboard");
    this.fieldOrientedEntry = table.getBooleanTopic("Field Oriented").getEntry(true);

    // Initializes the Sendables
    this.driveSpeedChooser = new SendableChooser<>();
    this.fieldOriented = new SendableChooser<>();

    // Sets the default value of the drive speed ratio. Without this, the robot
    // won't move because it has not value to use to calculate its speed.
    this.driveSpeedChooser.setDefaultOption("Full Speed", 1.0);
    // Adds additional options for the drive speed ratio.
    this.driveSpeedChooser.addOption("Three-Quarter Speed", 0.75);
    this.driveSpeedChooser.addOption("Half Speed", 0.5);
    this.driveSpeedChooser.addOption("Quarter Speed", 0.25);
    this.driveSpeedChooser.addOption("No Speed", 0.0);

    // Sets the default drive to Field Oriented. Without a default here, the robot
    // will not enable correctly.
    this.fieldOriented.setDefaultOption("Field Oriented", true);
    // Adds Robot Oriented as an option.
    this.fieldOriented.addOption("Robot Oriented", false);

    this.leftReefTargetPose.setDefaultOption("Original", 0.0);
    this.leftReefTargetPose.addOption("0.01", 0.01);
    this.leftReefTargetPose.addOption("0.02", 0.02);
    this.leftReefTargetPose.addOption("0.03", 0.03);
    this.leftReefTargetPose.addOption("-0.01", -0.01);
    this.leftReefTargetPose.addOption("-0.02", -0.02);
    this.leftReefTargetPose.addOption("-0.03", -0.03);

    this.rightReefTargetPose.setDefaultOption("Original", 0.32);
    this.rightReefTargetPose.addOption("0.01", 0.33);
    this.rightReefTargetPose.addOption("0.02", 0.34);
    this.rightReefTargetPose.addOption("0.03", 0.35);
    this.rightReefTargetPose.addOption("-0.01", 0.31);
    this.rightReefTargetPose.addOption("-0.02", 0.30);
    this.rightReefTargetPose.addOption("-0.03", 0.29);

    // Putting Sendables on the dashboard so they can be used.
    SmartDashboard.putData("Drive Speed", this.driveSpeedChooser);
    SmartDashboard.putData("Drive Orientation", this.fieldOriented);

    // Putting the field on the dashboard
    SmartDashboard.putData("Field", this.field);

    RobotConfig config;

    config = Constants.DriveConstants.RobotConfigInfo.robotConfig;

    AutoBuilder.configure(
        this::getRobotPose,
        this::resetPose,
        this::getCurrentSpeeds,
        this::drive,
        new PPHolonomicDriveController(
            new PIDConstants(TranslationPID.p, TranslationPID.i, TranslationPID.d),
            new PIDConstants(RotationPID.p, RotationPID.i, RotationPID.d)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Blue;
          } else {
            return false;
          }
        },
        this);

    this.gyro.reset();
  }

  //
  //
  public double getFieldAngle() {
    return -this.gyro.getAngle();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(this.getFieldAngle());
  }

  public void fieldOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, rot,
        Rotation2d.fromDegrees(this.getFieldAngle()));
    this.drive(speeds);
  }

  public void robotOrientedDrive(double speedX, double speedY, double rot) {
    ChassisSpeeds speeds = new ChassisSpeeds(speedX, speedY, rot);
    this.drive(speeds);
  }

  public boolean getDefaultDrive() {
    return this.fieldOriented.getSelected();
  }

  public void defaultDrive(double speedX, double speedY, double rot, boolean slew) {

    if (this.fieldOrientedEntry.get(this.getDefaultDrive())) {
      this.fieldOrientedDrive(speedX, speedY, rot);
    } else {
      this.robotOrientedDrive(speedX, speedY, rot);
    }
  }

  private void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
        DriveConstants.maxVelocity);

    this.frontLeft.drive(moduleStates[0]);
    this.frontRight.drive(moduleStates[1]);
    this.backLeft.drive(moduleStates[2]);
    this.backRight.drive(moduleStates[3]);
  }

  public double getDriverMaxVelocity() {
    return DriveConstants.maxVelocity * this.getRobotSpeedRatio();
  }

  public double getTrueMaxVelocity() {
    return DriveConstants.maxVelocity;
  }

  public double getMaxAngularVelocity() {
    return DriveConstants.maxAngularVelocity;
  }

  public Pose2d getRobotPose() {
    return this.odometry.getPoseMeters();
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[this.modules.length];
    for (int i = 0; i < this.modules.length; i++) {
      positions[i] = this.modules[i].getPosition();
    }
    return positions;
  }

  // Gets the currently selected ratio for the speed, as chosen on the driver
  // station dashboard.
  public double getRobotSpeedRatio() {
    return this.driveSpeedChooser.getSelected();
  }

  public void resetGyro() {
    this.gyro.reset();
  }

  public void resetPose(Pose2d pose2d) {
    this.odometry.resetPosition(this.getRotation2d(), this.getModulePositions(), pose2d);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public Command resetGyroCommand() {
    return this.runOnce(() -> this.resetGyro());
  }

  @SuppressWarnings("unused")
  private void saveGyroYaw() {
    this.savedGyroYaw = this.getFieldAngle();
  }

  @SuppressWarnings("unused")
  private void saveAutoYaw() {
    this.savedAutoYaw = this.getFieldAngle();
  }

  @SuppressWarnings("unused")
  private Pose2d endAutoPose() {
    Pose2d endPose = new Pose2d(
        this.odometry.getPoseMeters().getX(),
        this.odometry.getPoseMeters().getY(),
        new Rotation2d(this.savedGyroYaw + this.getFieldAngle() - this.savedAutoYaw));

    return endPose;
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */

    // Updating odometry
    var positions = this.getModulePositions();
    this.odometry.update(this.getRotation2d(), positions);

    // Update the double array storing the field pose by getting the values from the
    // Subscriber.
    this.R_limelightRobotPoseArray = this.R_limelightRobotPose.get();

    // Everything below is unnecessary for running the robot
  }
}
