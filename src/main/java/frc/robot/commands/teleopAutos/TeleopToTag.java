// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;
import swervelib.math.SwerveMath;

public class TeleopToTag extends Command {

  private final SwerveSubsystem m_swerve;

  private final LimelightVision m_llv;

  private final CommandXboxController m_controller;

  double angleKp = .001;

  double forwardkP = .1;

  public TeleopToTag(SwerveSubsystem swerve, LimelightVision llv, CommandXboxController controller) {
    m_swerve = swerve;
    m_llv = llv;
    m_controller = controller;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV(m_llv.frontname)) {

      double angleError = LimelightHelpers.getTX(m_llv.frontname);

      m_swerve.drive(
          new Translation2d(
              forwardToTag(),
              0),
          angleError * angleKp,
          false,
          true);
    }

    else {

      DoubleSupplier translationX = () -> -MathUtil.applyDeadband(
          m_controller.getLeftY(),
          OperatorConstants.LEFT_Y_DEADBAND);

      DoubleSupplier translationY = () -> -MathUtil.applyDeadband(
          m_controller.getLeftX(),
          OperatorConstants.DEADBAND);

      DoubleSupplier angularRotationX = () -> -MathUtil.applyDeadband(m_controller.getRightX(),
          OperatorConstants.RIGHT_X_DEADBAND);

      m_swerve.drive(
          SwerveMath.scaleTranslation(new Translation2d(
              translationX.getAsDouble() * m_swerve.swerveDrive.getMaximumChassisVelocity(),
              translationY.getAsDouble() * m_swerve.swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * m_swerve.swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);

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

  double forwardToTag() {

    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * forwardkP;
    targetingForwardSpeed *= m_swerve.swerveDrive.getMaximumChassisVelocity();
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

}
