// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory.AlgaeRPMSetpoints;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectAlgaeWhileIntaking extends Command {

  private final GamepieceSubsystem m_gamepieces;
  private MedianFilter sampleFilter;
  private MedianFilter detectFilter;
  private int algaeDetectLevel = 20;
  private int sampleFilterLevel = 5;
  private int sampleCount;
  private final int numberSamplesWanted = 25;// 1 second
  private int detectCount;
  private final int numberDetectsWanted = 25;// 1 second
  private double filteredRPM;
  private boolean algaeDetected;

  private double sampledRPM;

  public DetectAlgaeWhileIntaking(GamepieceSubsystem gamepieces) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepieces = gamepieces;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sampleFilter = new MedianFilter(sampleFilterLevel);
    detectFilter = new MedianFilter(algaeDetectLevel);
    sampleCount = 0;
    detectCount = 0;
    sampledRPM = 0;
    filteredRPM = 0;
    algaeDetected = false;
    sampleFilter.reset();
    detectFilter.reset();
    m_gamepieces.disableLimitSwitch();
    m_gamepieces.setCurrentLimit(20);
    m_gamepieces.motorLocked = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_gamepieces.run(AlgaeRPMSetpoints.kReefPickUpL123);

    sampleCount++;
    if (sampleCount <= numberSamplesWanted)
      sampledRPM = sampleFilter.calculate(-m_gamepieces.getRPM());

    else {
      filteredRPM = detectFilter.calculate(-m_gamepieces.getRPM());
      detectCount++;
    }

    algaeDetected = detectCount > numberDetectsWanted && filteredRPM < sampledRPM * m_gamepieces.getAlgaeDetectLevel();

    SmartDashboard.putNumber("Algae/FIlteredRPM", filteredRPM);
    SmartDashboard.putNumber("Algae/SampledRPM", sampledRPM);
    SmartDashboard.putBoolean("Algae/Detected", algaeDetected);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gamepieces.lockMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeDetected;
  }
}
