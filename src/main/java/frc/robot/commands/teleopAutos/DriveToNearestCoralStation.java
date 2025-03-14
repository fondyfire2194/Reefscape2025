// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopAutos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Side;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestCoralStation extends Command {
  /** Creates a new FindRobotReefZone. */
  SwerveSubsystem m_swerve;
  LimelightVision m_llv;
  boolean exit;
  int tst;
  Side m_side;
  boolean setSide;
  Translation2d tl2d;
  Constraints driveConstraints = new Constraints(3.5, 5);

  

  public DriveToNearestCoralStation(SwerveSubsystem swerve) {
    m_swerve = swerve;
    setSide = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tst = 0;
    exit = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = new Pose2d();

    if (!m_swerve.lockPoseChange) {

      m_swerve.driveToPose(m_swerve.coralStationFinalTargetPose).schedule();

      // new DriveToPointCommand(m_swerve).schedule();

      exit = true;
    } else
      tst++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_llv.getTXOKDeliverCoral();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exit || tst > 2;
  }
}
