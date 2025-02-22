// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.utils.LimelightHelpers;
import monologue.Annotations.Log;
import monologue.Logged;

public class LimelightVision extends SubsystemBase implements Logged {
  /** Creates a new LimelightVision. */

  public boolean limelightExistsfront;

  public boolean limelightExistsleft;

  public boolean limelightExistsright;

  public boolean limelightExistsrear;

  private int loopctr;

  public String frontname = VisionConstants.CameraConstants.frontCamera.camname;

  Optional<Pose3d> temp;

  final int[] autoTagFilter = new int[] { 10, 11, 6, 7, 8, 9, 21, 22, 17, 18, 19, 20 };

  Alert flCameraAlert = new Alert("FrontCameraProblem", AlertType.kWarning);
  Alert frCameraAlert = new Alert("FrontRightCameraProblem", AlertType.kError);
  Alert rearCameraAlert = new Alert("RearCameraProblem", AlertType.kInfo);
  @Log(key = "flaccpose")
  public static Pose2d flAcceptedPose;
  @Log(key = "fraccpose")
  public static Pose2d frAcceptedPose;
  @Log(key = "rearaccpose")
  public static Pose2d rearAcceptedPose;
  @Log(key = "flaccepted")
  public static int flAcceptedCount;
  @Log(key = "fraccepted")
  public static int frAcceptedCount;
  @Log(key = "rearaccepted")
  public static int rearAcceptedCount;

  /**
   * Checks if the specified limelight is connected
   *
   * @param limelight A limelight (BACK, FRONT_LEFT, FRONT_RIGHT).
   * @return True if the limelight network table contains the key "tv"
   */
  public boolean isLimelightConnected(String camname) {
    return LimelightHelpers.getLimelightNTTable(camname).containsKey("tv");
  }

  public LimelightVision() {

    if (VisionConstants.CameraConstants.frontCamera.isUsed) {
      setCamToRobotOffset(VisionConstants.CameraConstants.frontCamera);
    }

    if (VisionConstants.CameraConstants.rightCamera.isUsed)
      setCamToRobotOffset(VisionConstants.CameraConstants.rightCamera);

  }

  public void setAprilTagFilter(String camname) {
    LimelightHelpers.SetFiducialIDFiltersOverride(camname, autoTagFilter);
  }

  public Command clearPOI() {
    return Commands
        .runOnce(() -> LimelightHelpers.SetFidcuial3DOffset(frontname, 0, 0, 0));
  }

  public Command setPOIRight() {
    return Commands
        .runOnce(() -> LimelightHelpers.SetFidcuial3DOffset(frontname, FieldConstants.centerToReefBranch, 0, 0));
  }

  public Command setPOILeft() {
    return Commands
        .runOnce(() -> LimelightHelpers.SetFidcuial3DOffset(frontname, -FieldConstants.centerToReefBranch, 0, 0));
  }

  public double getDistanceToTag(String camname) {
    if (LimelightHelpers.getTV(camname)) {
      Pose3d tag3dpose = LimelightHelpers.getTargetPose3d_CameraSpace(camname);
      return tag3dpose.getTranslation().getNorm();
    } else
      return 0;
  }

  public boolean getTXOKDeliverCoral() {
    double tx = LimelightHelpers.getTX(frontname);
    double distance = Units.metersToInches(getDistanceToTag(frontname));
    double yerror = Math.tan(Units.degreesToRadians(tx)) * distance;
    return yerror < 2;
  }

  @Override
  public void periodic() {

    if (RobotBase.isReal()) {
      if (VisionConstants.CameraConstants.frontCamera.isUsed && loopctr == 0) {
        limelightExistsfront = isLimelightConnected(CameraConstants.frontCamera.camname);
        limelightExistsleft = isLimelightConnected(CameraConstants.rightCamera.camname);
        limelightExistsrear = isLimelightConnected(CameraConstants.rearCamera.camname);

        boolean allcamsok = VisionConstants.CameraConstants.frontCamera.isUsed && limelightExistsfront
            && VisionConstants.CameraConstants.rightCamera.isUsed && limelightExistsleft
            && VisionConstants.CameraConstants.rearCamera.isUsed && limelightExistsrear;
        SmartDashboard.putBoolean("LL//CamsOK", allcamsok);
      }

      SmartDashboard.putBoolean("LL//FrontLeftCamOk", limelightExistsfront);
      SmartDashboard.putBoolean("LL//FrontRightCamOk", limelightExistsleft);
      SmartDashboard.putBoolean("LL//RearCamOk", limelightExistsrear);
    }
  }

  public void setCamToRobotOffset(VisionConstants.CameraConstants.CameraValues cam) {
    LimelightHelpers.setCameraPose_RobotSpace(cam.camname, cam.forward, cam.side, cam.up, cam.roll, cam.pitch, cam.yaw);
  }

}