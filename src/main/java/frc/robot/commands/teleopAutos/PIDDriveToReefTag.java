// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

/*
 * Test code for driving to reef april tag,
 * Readings are done and adjustments made throught the motion
 * Uses tx from the limelight to strafe
 * Uses distance from tag as calculated by limelight
 * Uses robot pose angle for target angle
 * Compenstaes using limleight time valuse
 * 
 * Sets april tag of interest in Limelight
 * Sets Point of Interest for left, center and right so limelight shifts its 0 point
 * 
 * Adjusts the PID targets from calculations
 * 
 * 
 */

public class PIDDriveToReefTag extends Command {
  private final SwerveSubsystem m_swerve;
  private Pose2d target;

  private final PIDController xController = new PIDController(1.9, 0, 0);
  private final PIDController yController = new PIDController(1.9, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);
  private final LimelightVision m_llv;

  /** Creates a new TurnToRelativeAngleTrapezoidProfile. */
  public PIDDriveToReefTag(SwerveSubsystem swerve, LimelightVision llv) {
    m_swerve = swerve;
    m_llv = llv;

    xController.setTolerance(0.025);// 1"
    yController.setTolerance(0.025);// 1"
    thetaController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());// .5 degrees

    int reefZone = swerve.reefZone;
    int tagNumber = FieldConstants.blueReefTags[reefZone];
    Pose2d finalTarget = swerve.getFinalReefTargetPose();
    target=finalTarget;
    double poseDistancceToTarget = finalTarget.getTranslation().getDistance(swerve.getPose().getTranslation());
    double cameraDistanceToTarget = m_llv.getDistanceToTag(CameraConstants.frontCamera.camname);
    double distanceDifference = poseDistancceToTarget - cameraDistanceToTarget;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    thetaController.setSetpoint(target.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    int tagseen = (int) LimelightHelpers.getFiducialID(CameraConstants.frontCamera.camname);

    double tx = LimelightHelpers.getTX(CameraConstants.frontCamera.camname);
    double llxkp = .001;
    double yTarget = yController.getSetpoint() + tx * llxkp;
    yController.setSetpoint(yTarget);
    m_swerve.drive(
        new Translation2d(
            xController.calculate(m_swerve.getPose().getX()),
            yController.calculate(m_swerve.getPose().getY())),
        thetaController.calculate(m_swerve.getPose().getRotation().getRadians()),
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
