// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class BackupOffSwitchAfterCoraIntake extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private Timer backupTimer;

  public BackupOffSwitchAfterCoraIntake(GamepieceSubsystem gamepiece) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
    backupTimer = new Timer();
    backupTimer.reset();
    backupTimer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamepiece.disableLimitSwitch();
    m_gamepiece.setCurrentLimit(m_gamepiece.inOutCoralAmps);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_gamepiece.gamepieceMotor.set(-.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gamepiece.stopGamepieceMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_gamepiece.coralAtIntake() || backupTimer.hasElapsed(2);

  }
}
