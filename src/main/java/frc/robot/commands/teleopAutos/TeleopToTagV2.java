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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class TeleopToTagV2 extends Command {

  private final SwerveSubsystem m_swerve;

  private final LimelightVision m_llv;

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

  public TeleopToTagV2(SwerveSubsystem swerve, LimelightVision llv) {
    m_swerve = swerve;
    m_llv = llv;
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

    Pose2d poseOffset = new Pose2d();

    if (!LimelightHelpers.getTV(m_llv.frontname))
      tagPose = m_swerve.getFinalReefTargetPose();
    else
      tagPose = LimelightHelpers.getTargetPose3d_RobotSpace(m_llv.frontname).toPose2d();

    /**
     * Robot Space
     * 3d Cartesian Coordinate System with (0,0,0) located at the center of the
     * robot’s frame projected down to the floor.
     * 
     * X+ → Pointing forward (Forward Vector)
     *
     * Y+ → Pointing toward the robot’s right (Right Vector)
     *
     * Z+ → Pointing upward (Up Vector)
     * 
     * 
     */

    double baseOffset = RobotConstants.placementOffset + RobotConstants.ROBOT_LENGTH / 2;
    if (m_swerve.side == Side.CENTER)
      tl2d = new Translation2d(baseOffset, 0);
    if (m_swerve.side == Side.RIGHT)
      tl2d = new Translation2d(baseOffset, FieldConstants.reefOffset);
    if (m_swerve.side == Side.LEFT)
      tl2d = new Translation2d(baseOffset, -FieldConstants.reefOffset);
    Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));
    tagPose.transformBy(tr2d);

    m_swerve.poseTagActive = tagPose;

    Pose2d currentPose = m_swerve.getPose();
    x = controllerX.calculate(currentPose.getX(), tagPose.getX());
    y = controllerY.calculate(currentPose.getY(), tagPose.getY());
    rotation = controllerRotation.calculate(currentPose.getRotation().getRadians(), tagPose.getRotation().getRadians());

    x = MathUtil.clamp(x, minSet, maxSet);
    y = MathUtil.clamp(y, minSet, maxSet);
    rotation = MathUtil.clamp(rotation, -rotLimit, rotLimit);

    // SmartDashboard.putNumber("TTT/X-X", tagPose.getX()-currentPose.getX());
    // SmartDashboard.putNumber("TTT/X", x);
    // SmartDashboard.putNumber("TTT/Y", y);
    // SmartDashboard.putNumber("TTT/Rot", rotation);

    poseOffset = new Pose2d(x, y, new Rotation2d(rotation));

    m_swerve.poseTagActive = poseOffset;

    // SmartDashboard.putNumber("TTT/XVEL", x *
    // m_swerve.swerveDrive.getMaximumChassisVelocity());
    // SmartDashboard.putNumber("TTT/YVEL", y *
    // m_swerve.swerveDrive.getMaximumChassisVelocity());
    // SmartDashboard.putNumber("TTT/RotVEL", rotation *
    // m_swerve.swerveDrive.getMaximumChassisAngularVelocity());

    m_swerve.drive(
        new Translation2d(
            poseOffset.getX() * m_swerve.swerveDrive.getMaximumChassisVelocity(),
            poseOffset.getY() * m_swerve.swerveDrive.getMaximumChassisVelocity()),
        poseOffset.getRotation().getRadians() * m_swerve.swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        true);

  }

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
