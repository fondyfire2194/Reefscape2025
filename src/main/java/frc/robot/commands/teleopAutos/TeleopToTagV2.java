// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class TeleopToTagV2 extends Command {

  private final SwerveSubsystem m_swerve;
  private final LimelightVision m_llv;
  private final CommandXboxController m_gamepad;
  Pose2d tagPose;
  Translation2d tl2d = new Translation2d();
  private PIDController controllerX;
  private PIDController controllerY;
  private PIDController controllerRotation;

  double x;
  double y;
  double rotation;
  double maxSet = .5;
  double minSet = -.5;
  double rotLimit = .5;
  boolean showTelemetry = true;

  public TeleopToTagV2(SwerveSubsystem swerve, LimelightVision llv, CommandXboxController gamepad) {
    m_swerve = swerve;
    m_llv = llv;
    m_gamepad = gamepad;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerX = new PIDController(.7, 0.0, 0.0);
    controllerY = new PIDController(.7, 0.0, 0.0);
    controllerRotation = new PIDController(3.0, 0.0, 0.0);
    controllerRotation.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double triggerSetting = m_gamepad.getLeftTriggerAxis();

    Pose2d poseOffset = new Pose2d();

    if (!LimelightHelpers.getTV(m_llv.frontname))
      tagPose = m_swerve.getFinalReefTargetPose();
    else
      tagPose = LimelightHelpers.getBotPose3d_wpiBlue(m_llv.frontname).toPose2d();

    double baseOffsetX = RobotConstants.placementOffsetX + RobotConstants.ROBOT_LENGTH / 2;
    double baseOffsetY = RobotConstants.placementOffsetY;

    if (m_swerve.side == Side.CENTER)
      tl2d = new Translation2d(baseOffsetX, baseOffsetY);
    if (m_swerve.side == Side.RIGHT)
      tl2d = new Translation2d(baseOffsetX, FieldConstants.reefOffset + baseOffsetY);
    if (m_swerve.side == Side.LEFT)
      tl2d = new Translation2d(baseOffsetX, -FieldConstants.reefOffset + baseOffsetY);
    Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

    tagPose.transformBy(tr2d);
    m_swerve.poseTagActive = tagPose;

    Pose2d currentPose = m_swerve.getPose();
    x = controllerX.calculate(currentPose.getX(), tagPose.getX());
    y = controllerY.calculate(currentPose.getY(), tagPose.getY());
    rotation = controllerRotation.calculate(currentPose.getRotation().getRadians(),
        tagPose.getRotation().getRadians());

    x = MathUtil.clamp(x, minSet, maxSet);
    y = MathUtil.clamp(y, minSet, maxSet);
    rotation = MathUtil.clamp(rotation, -rotLimit, rotLimit);

    poseOffset = new Pose2d(x, y, new Rotation2d(rotation));
    m_swerve.poseTagActive = poseOffset;

    if (showTelemetry) {
      SmartDashboard.putNumber("TTT/X-X", tagPose.getX() - currentPose.getX());
      SmartDashboard.putNumber("TTT/X", x);
      SmartDashboard.putNumber("TTT/Y", y);
      SmartDashboard.putNumber("TTT/Rot", rotation);
      SmartDashboard.putNumber("TTT/XVEL", x *
          m_swerve.swerveDrive.getMaximumChassisVelocity());
      SmartDashboard.putNumber("TTT/YVEL", y *
          m_swerve.swerveDrive.getMaximumChassisVelocity());
      SmartDashboard.putNumber("TTT/RotVEL", rotation *
          m_swerve.swerveDrive.getMaximumChassisAngularVelocity());
    }
    m_swerve.drive(
        new Translation2d(
            poseOffset.getX() * m_swerve.swerveDrive.getMaximumChassisVelocity(),
            poseOffset.getY() * m_swerve.swerveDrive.getMaximumChassisVelocity()),
        poseOffset.getRotation().getRadians() * m_swerve.swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        true);

  }
  // from CD
  // ChassisSpeeds speeds = mDriveController.calculate(mPoseEstimator.getPose(),
  // mGoal, 0, mGoal.getRotation());
  // mSwerve.setModuleStates(SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds));

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
