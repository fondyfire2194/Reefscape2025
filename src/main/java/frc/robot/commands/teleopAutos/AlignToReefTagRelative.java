// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;

  // Auto constants
	public static final double X_REEF_ALIGNMENT_P = 2.5;
	public static final double Y_REEF_ALIGNMENT_P = 4.5;
	public static final double ROT_REEF_ALIGNMENT_P = 0.058;

	public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.5;
	public static final double X_SETPOINT_REEF_ALIGNMENT = -0.5;  // Vertical pose
	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;
	public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.19;  // Horizontal pose
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.005;

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

  public AlignToReefTagRelative(boolean isRightScore, SwerveSubsystem drivebase) {
    xController = new PIDController(X_REEF_ALIGNMENT_P, 0, 0);  // Vertical movement
    yController = new PIDController(Y_REEF_ALIGNMENT_P, 0, 0);  // Horitontal movement
    rotController = new PIDController(ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Y_SETPOINT_REEF_ALIGNMENT : -Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("")) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", postions[2]);

      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspee", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      double rotValue = -rotController.calculate(postions[4]);

      drivebase.drive(new Translation2d(yController.getError() < Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(POSE_VALIDATION_TIME);
  }
}