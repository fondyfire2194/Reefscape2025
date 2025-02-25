// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import swervelib.math.SwerveMath;

public class TeleopToTagV2 extends Command {

  private final SwerveSubsystem m_swerve;

  private final LimelightVision m_llv;

  private final CommandXboxController m_controller;

  double angleKp = .001;

  double forwardkP = .1;

  Pose2d tagPose;

  private PIDController controllerX;
  private PIDController controllerY;
  private PIDController controllerRotation;
  double x;
  double y;
  double rotation;
  double maxSet = .25;
  double minSet = -.25;
  double rotLimit = .5;

  public TeleopToTagV2(SwerveSubsystem swerve, LimelightVision llv, CommandXboxController controller) {
    m_swerve = swerve;
    m_llv = llv;
    m_controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controllerX = new PIDController(1.7, 0.0, 0.0);
    controllerY = new PIDController(1.7, 0.0, 0.0);
    controllerRotation = new PIDController(2.0, 0.0, 0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d poseOffset = new Pose2d();

    if (RobotBase.isSimulation() || LimelightHelpers.getTV(m_llv.frontname)) {
      if (RobotBase.isSimulation())
        tagPose = m_swerve.getFinalReefTargetPose();
      else
        tagPose = LimelightHelpers.getTargetPose3d_RobotSpace(m_llv.frontname).toPose2d();

      m_swerve.poseTagActive = tagPose;

      Pose2d currentPose = m_swerve.getPose();
      x = controllerX.calculate(currentPose.getX(), tagPose.getX());
      y = controllerY.calculate(currentPose.getY(), tagPose.getY());
      rotation = controllerRotation.calculate(currentPose.getRotation().getRadians(),
          tagPose.getRotation().getRadians());

      x = ensureRange(x, minSet, maxSet);
      y = ensureRange(y, minSet, maxSet);
      rotation = ensureRange(rotation, minSet, maxSet);

      SmartDashboard.putNumber("TTT/X", x);
      SmartDashboard.putNumber("TTT/Y", y);
      SmartDashboard.putNumber("TTT/Rot", rotation);

      if (Math.abs(controllerRotation.getError()) > rotLimit) {
        poseOffset = new Pose2d(0, 0, new Rotation2d(rotation));
      } else {
        poseOffset = new Pose2d(x, y, new Rotation2d(rotation));
      }

      SmartDashboard.putNumber("TTT/XVEL", x * m_swerve.swerveDrive.getMaximumChassisVelocity());
      SmartDashboard.putNumber("TTT/YVEL", y * m_swerve.swerveDrive.getMaximumChassisVelocity());
      SmartDashboard.putNumber("TTT/RotVEL", rotation * m_swerve.swerveDrive.getMaximumChassisAngularVelocity());

      m_swerve.drive(
          new Translation2d(poseOffset.getX() * m_swerve.swerveDrive.getMaximumChassisVelocity(),
              poseOffset.getY() * m_swerve.swerveDrive.getMaximumChassisVelocity()),
          poseOffset.getRotation().getRadians() * m_swerve.swerveDrive.getMaximumChassisAngularVelocity(),
          false,
          true);
    } else {

      DoubleSupplier translationX = () -> -MathUtil.applyDeadband(
          m_controller.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND);

      DoubleSupplier translationY = () -> -MathUtil.applyDeadband(
          m_controller.getLeftX(),
          OperatorConstants.DEADBAND);

      DoubleSupplier angularRotationX = () -> -MathUtil.applyDeadband(m_controller.getRightX(),
          OperatorConstants.RIGHT_X_DEADBAND);

      m_swerve.drive(
          SwerveMath.scaleTranslation(
              new Translation2d(translationX.getAsDouble() * m_swerve.swerveDrive.getMaximumChassisVelocity(),
                  translationY.getAsDouble() * m_swerve.swerveDrive.getMaximumChassisVelocity()),
              0.8),
          Math.pow(angularRotationX.getAsDouble(), 3)
              * m_swerve.swerveDrive.getMaximumChassisAngularVelocity(),
          true, false);

    }
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

  double ensureRange(double value, double min, double max) {
    return Math.min(Math.max(value, min), max);
  }
}
