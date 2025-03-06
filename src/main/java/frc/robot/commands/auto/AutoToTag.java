// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.Optional;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class AutoToTag extends Command {
  private final SwerveSubsystem m_swerve;

  private final LimelightVision m_llv;

  Pose2d tagPose2d = new Pose2d();
  double angleKp = .001;
  double strafekP = .035;
  double forwardkP = .1;

  public AutoToTag(SwerveSubsystem swerve, LimelightVision llv) {
    m_swerve = swerve;
    m_llv = llv;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PPHolonomicDriveController.clearFeedbackOverrides();
    if (LimelightHelpers.getTV(m_llv.frontname)) {
      int tagNum = (int) LimelightHelpers.getFiducialID(m_llv.frontname);
      Optional<Pose3d> tagPoseOpt3d = m_swerve.aprilTagFieldLayout.getTagPose(tagNum);
      if (tagPoseOpt3d.isPresent())
        tagPose2d = tagPoseOpt3d.get().toPose2d();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV(m_llv.frontname)) {

      double angleError = tagPose2d.getRotation().getDegrees() - m_swerve.getPose().getRotation().getDegrees();

      double yError = LimelightHelpers.getTX(m_llv.frontname);

      PPHolonomicDriveController.overrideXFeedback(() -> yError * strafekP);
      PPHolonomicDriveController.overrideRotationFeedback(() -> angleError * strafekP);
    }

    else {
      PPHolonomicDriveController.clearFeedbackOverrides();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PPHolonomicDriveController.clearFeedbackOverrides();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

}
