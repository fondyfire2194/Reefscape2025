// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import frc.robot.Factories.CommandFactory.CoralRPMSetpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private final ArmSubsystem m_arm;
  private final boolean m_usePIDSpeed;
  private Timer noCoralTimer;

  public IntakeCoralToSwitch(GamepieceSubsystem gamepiece, ArmSubsystem arm, boolean usePIDSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
    m_arm = arm;
    m_usePIDSpeed = usePIDSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noCoralTimer = new Timer();
    noCoralTimer.reset();
    noCoralTimer.start();
    m_gamepiece.enableLimitSwitch();
    m_gamepiece.setCurrentLimit(m_gamepiece.inOutCoralAmps);
    if (!m_usePIDSpeed) {
      m_gamepiece.gamepieceMotor.set(.25);
      m_gamepiece.coralIntakeMotor.set(.35);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_usePIDSpeed) {
      m_gamepiece.runGamepieceMotorAtVelocity(CoralRPMSetpoints.kGmepieceCoralIntakeRPM);
      m_gamepiece.runCoralIntakeMotorAtVelocity(CoralRPMSetpoints.kCoralIntakeMotorRPM);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (!noCoralTimer.hasElapsed(m_gamepiece.noCoralAtSwitchTime))
      m_arm.setGoalDegrees(ArmSetpoints.kTravel);
    m_gamepiece.stopCoralIntakeMotor();
    m_gamepiece.stopGamepieceMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtIntake() || noCoralTimer.hasElapsed(m_gamepiece.noCoralAtSwitchTime);

  }
}
