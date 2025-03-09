// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PIDDriveToPose extends Command {
  private final SwerveSubsystem swerve;
  private Pose2d target;

  private final PIDController xController = new PIDController(2.5, 0, 0);
  private final PIDController yController = new PIDController(2.5, 0, 0);
  private final PIDController thetaController = new PIDController(3, 0, 0);

  /** Creates a new PIDDriveToPose. */
  public PIDDriveToPose(SwerveSubsystem swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    thetaController.setTolerance(Rotation2d.fromDegrees(0.5).getRadians());
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Translation2d tl2d = new Translation2d();
    Side m_side = swerve.side;

    double baseOffsetX = RobotConstants.placementOffsetX + RobotConstants.ROBOT_LENGTH / 2;
    double baseOffsetY = RobotConstants.placementOffsetY;
    if (m_side == Side.CENTER)
      tl2d = new Translation2d(baseOffsetX, baseOffsetY);
    if (m_side == Side.RIGHT)
      tl2d = new Translation2d(baseOffsetX, FieldConstants.reefOffset + baseOffsetY);
    if (m_side == Side.LEFT)
      tl2d = new Translation2d(baseOffsetX, -FieldConstants.reefOffset + baseOffsetY);

    Transform2d tr2d = new Transform2d(tl2d, new Rotation2d(Units.degreesToRadians(180)));

    target = target.transformBy(tr2d);
    swerve.reefFinalTargetPose = target;

    xController.setSetpoint(target.getX());
    yController.setSetpoint(target.getY());
    thetaController.setSetpoint(target.getRotation().getRadians());
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
