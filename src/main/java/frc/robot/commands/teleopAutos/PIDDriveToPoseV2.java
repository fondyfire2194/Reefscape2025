// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PIDDriveToPoseV2 extends Command {
  private final SwerveSubsystem swerve;
  private final Pose2d target;

  double TRAJECTORY_VEL = 1;
  double TRAJECTORY_ACCEL = 2;

  double ROT_VEL = 1;
  double ROT_ACCEL = 2;

  private TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      TRAJECTORY_VEL, TRAJECTORY_ACCEL);
  private TrapezoidProfile.Constraints rotConstraints = new TrapezoidProfile.Constraints(
      ROT_VEL, ROT_ACCEL);

  double xKp = 2;
  double yKp = 2;
  double rotKp = 3;

  private final ProfiledPIDController xController = new ProfiledPIDController(xKp, 0, 0, constraints,.02);
  private final ProfiledPIDController yController = new ProfiledPIDController(yKp, 0, 0, constraints,.02);
  private final ProfiledPIDController thetaController = new ProfiledPIDController(rotKp, 0, 0, rotConstraints,.02);

  /** Creates a new PIDDriveToPose. */
  public PIDDriveToPoseV2(SwerveSubsystem swerve, Pose2d target) {
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
    xController.setGoal(target.getX());
    yController.setGoal(target.getY());
    thetaController.setGoal(target.getRotation().getRadians());
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
