// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectAlgaeWhileIntaking extends Command {

  private final AlgaeIntakeSubsystem m_algae;

  private MedianFilter sampleFilter;
  private MedianFilter detectFilter;
  private int algaeDetectLevel = 20;
  private int sampleFilterLevel = 5;
  private int sampleCount;
  private final int numberSamplesWanted = 50;// 1 second
  private int detectCount;
  private final int numberDetectsWanted = 25;// 1 second
  private double filteredRPM;
  private boolean algaeDetected;

  public DetectAlgaeWhileIntaking(AlgaeIntakeSubsystem algae) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_algae = algae;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sampleFilter = new MedianFilter(sampleFilterLevel);
    detectFilter = new MedianFilter(algaeDetectLevel);
    sampleCount = 0;
    detectCount = 0;
    algaeDetected = false;
    sampleFilter.reset();
    detectFilter.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sampleCount++;
    if (sampleCount <= numberSamplesWanted)
      filteredRPM = sampleFilter.calculate(m_algae.getRPM());
    else
      filteredRPM = detectFilter.calculate(m_algae.getRPM());

    if (sampleCount > numberSamplesWanted && detectCount > numberDetectsWanted) {
      detectCount++;
      algaeDetected = detectCount > numberDetectsWanted && filteredRPM < filteredRPM;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_algae.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return algaeDetected;
  }
}
