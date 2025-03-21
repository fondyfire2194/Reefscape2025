// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import monologue.Logged;
import monologue.Monologue;
import monologue.Annotations.Log;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot implements Logged {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  @Log
  Pose2d result;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   * 
   * Radio SSID is Fondy2194
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    Monologue.setupMonologue(m_robotContainer, "/Monologue", false, true);

    DriverStation.startDataLog(DataLogManager.getLog());

    // URCL.start();

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();

    // Make sure you only configure port forwarding once in your robot code.
    // Do not place these function calls in any periodic functions
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    URCL.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // setFileOnly is used to shut off NetworkTables broadcasting for most logging
    // calls.
    // Basing this condition on the connected state of the FMS is a suggestion only.
    Monologue.setFileOnly(DriverStation.isFMSAttached());
    // This method needs to be called periodically, or no logging annotations will
    // process properly.
    Monologue.updateAll();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();

    m_robotContainer.drivebase.frontUpdate.setLLRobotorientation();
    m_robotContainer.drivebase.rearUpdate.setLLRobotorientation();
  }

  @Override
  public void disabledPeriodic() {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
    /**
     * 
     * targetpose_robotspace doubleArray 3D transform of the primary in-view
     * AprilTag in the coordinate system of the Robot
     * (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
     * 
     * botpose_targetspace doubleArray 3D transform of the robot in the coordinate
     * system of the primary in-view AprilTag
     * (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees)
     * 
     * 
     */

    boolean tagSeen = LimelightHelpers.getTV(CameraConstants.frontCamera.camname);

    SmartDashboard.putBoolean("AutoAlign/Tag Seen", tagSeen);

    if (tagSeen) {
      double targetDistance = 3;
      Pose2d p2d = LimelightHelpers.getTargetPose3d_RobotSpace(CameraConstants.frontCamera.camname).toPose2d();
      double distance = m_robotContainer.llv.getDistanceToTag(CameraConstants.frontCamera.camname);
      double[] data = new double[6];

      data = LimelightHelpers.pose2dToArray(p2d);

      double tx = data[0];
      // double ty = data[1];
      // double tz = data[2];
      // double pitch = data[3];
      // double roll = data[4];
      double yaw = data[5];

      SmartDashboard.putNumber("AutoAlign/TX", tx);
      SmartDashboard.putNumber("AutoAlign/Yaw", yaw);

      boolean txOnTarget = Math.abs(tx) < .1;
      boolean txClosePlus = tx > 0 && tx < 1;
      boolean txCloseMinus = tx < 0 && tx > -1;

      SmartDashboard.putBoolean("AutoAlign/TXOnTarget", txOnTarget);
      SmartDashboard.putBoolean("AutoAlign/TXClosePlus", txClosePlus);
      SmartDashboard.putBoolean("AutoAlign/TXCloseMinus", txCloseMinus);

      boolean yawOnTarget = Math.abs(yaw) < .1;
      boolean yawClosePlus = tx > 0 && tx < 1;
      boolean yawCloseMinus = tx < 0 && tx > -1;

      SmartDashboard.putBoolean("AutoAlign/YawOnTarget", yawOnTarget);
      SmartDashboard.putBoolean("AutoAlign/YawClosePlus", yawClosePlus);
      SmartDashboard.putBoolean("AutoAlign/YawCloseMINUS", yawCloseMinus);
      double distanceError = targetDistance - distance;
      boolean distanceOnTarget = Math.abs(distanceError) < .01;
      boolean distanceClosePlus = (distanceError) > 0 && (distanceError) < .01;
      boolean distanceCloseMinus = distanceError < 0 && distanceError > -1;

      SmartDashboard.putBoolean("AutoAlign/DistanceOnTarget", distanceOnTarget);
      SmartDashboard.putBoolean("AutoAlign/DistanceClosePlus", distanceClosePlus);
      SmartDashboard.putBoolean("AutoAlign/DistanceCloseMINUS", distanceCloseMinus);

    }
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.llv.setPOILeft();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }

    m_robotContainer.configureCoDriverTeleopBindings();

    m_robotContainer.drivebase.frontUpdate.setUseMegatag2(true);
    m_robotContainer.drivebase.rearUpdate.setUseMegatag2(true);
    m_robotContainer.llv.inhibitFrontVision = false;
    m_robotContainer.llv.inhibitRearVision = true;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_robotContainer.configureCoDriverTestBindings();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit() {

  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic() {
  }
}
