// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class PIDDriveToPoseTagOverride extends Command {
  private final SwerveSubsystem swerve;
  private final LimelightVision llv;
  private final Side side;
  private final int tag;
  private Pose2d target = new Pose2d();
  private final PIDController xController = new PIDController(1.9, 0, 0);
  private final PIDController yController = new PIDController(1.9, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);

  private Translation2d tl2d = new Translation2d();

  /** Creates a new PIDDriveToPose. */
  public PIDDriveToPoseTagOverride(SwerveSubsystem swerve, Side side, LimelightVision llv, int tag) {
    this.swerve = swerve;
    this.llv = llv;
    this.tag = tag;
    this.side = side;
    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Pose3d> tagPoseOpt3d = swerve.aprilTagFieldLayout.getTagPose(tag);
    if (tagPoseOpt3d.isPresent())
      target = tagPoseOpt3d.get().toPose2d();

    double baseOffsetX = RobotConstants.placementOffsetX + RobotConstants.ROBOT_LENGTH / 2;
    double baseOffsetY = RobotConstants.placementOffsetY;

    if (side == Side.CENTER) {
      tl2d = new Translation2d();
      llv.clearPOI();
    }
    if (side == Side.RIGHT) {
      tl2d = new Translation2d(baseOffsetX, baseOffsetY);
      llv.setPOIRight();
    }
    if (side == Side.LEFT) {
      tl2d = new Translation2d(baseOffsetX, -baseOffsetY);
      llv.setPOILeft();
    }

    Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

    swerve.reefFinalTargetPose = target.transformBy(tr2d);

    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    thetaController.setSetpoint(target.getRotation().getRadians());
    LimelightHelpers.setPriorityTagID(llv.frontname, tag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d tl2d = new Translation2d(xController.calculate(swerve.getPose().getX()),
        yController.calculate(swerve.getPose().getY()));

    swerve.drive(
        tl2d,
        thetaController.calculate(swerve.getPose().getRotation().getRadians()),
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
