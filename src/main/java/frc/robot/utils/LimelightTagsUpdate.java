// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.VisionConstants.CameraConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class LimelightTagsUpdate {

    private final SwerveSubsystem m_swerve;
    private final CameraConstants.CameraValues m_cam;
    private boolean m_useMegaTag2 = false;
    boolean rejectUpdate;

    public LimelightTagsUpdate(CameraConstants.CameraValues cam, SwerveSubsystem swerve) {
        m_cam = cam;
        m_swerve = swerve;

    }

    public void setUseMegatag2(boolean on) {
        m_useMegaTag2 = on;
    }

    public void setLLRobotorientation() {
        SmartDashboard.putNumber("LLGyroRead", m_swerve.getPose().getRotation().getDegrees());
        LimelightHelpers.SetRobotOrientation(m_cam.camname,
                m_swerve.getPose().getRotation().getDegrees(),
                // m_swerve.getHeadingDegrees(),
                m_swerve.getGyroRate(), 0, 0, 0, 0); // m_swerve.getPoseEstimator().getEstimatedPosition().getRotation().getDegrees()
    }

    public void execute() {

        if (m_cam.isActive && LimelightHelpers.getTV(m_cam.camname)) {
            setLLRobotorientation();
            if (m_useMegaTag2) {
                setLLRobotorientation();
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_cam.camname);
                m_swerve.distanceLimelightToEstimator = mt2.pose.getTranslation()
                        .getDistance(m_swerve.getPoseEstimator().getEstimatedPosition().getTranslation());

                rejectUpdate = mt2.tagCount == 0 || Math.abs(m_swerve.getGyroRate()) > 720
                        || (mt2.tagCount == 1 && mt2.rawFiducials.length == 1 &&
                                mt2.rawFiducials[0].ambiguity > .7
                                && mt2.rawFiducials[0].distToCamera > 4);

                SmartDashboard.putBoolean("RejectUpdateMT2" + m_cam.camname, rejectUpdate);

                SmartDashboard.putNumber("LLMT2 X", mt2.pose.getTranslation().getX());
                SmartDashboard.putNumber("LLMT2 Y", mt2.pose.getTranslation().getY());

                if (!rejectUpdate) {
                    double standard_devs = mt2.rawFiducials[0].distToCamera / 2;// 0.5
                    m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(standard_devs,
                            standard_devs, 9999999));
                    m_swerve.getPoseEstimator().addVisionMeasurement(
                            mt2.pose,
                            mt2.timestampSeconds);
                }

            } else {

                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_cam.camname);

                rejectUpdate = mt1.tagCount == 0
                        || mt1.tagCount == 1 && mt1.rawFiducials.length == 1 &&
                                mt1.rawFiducials[0].ambiguity > .7
                                && mt1.rawFiducials[0].distToCamera > 5;

                SmartDashboard.putBoolean("RejectUpdateMT1" + m_cam.camname, rejectUpdate);

                SmartDashboard.putNumber("LLMT1 X", mt1.pose.getTranslation().getX());
                SmartDashboard.putNumber("LLMT1 Y", mt1.pose.getTranslation().getY());

                if (!rejectUpdate) {
                    m_swerve.getPoseEstimator().setVisionMeasurementStdDevs(VecBuilder.fill(.7,
                            .7, 1));
                    m_swerve.getPoseEstimator().addVisionMeasurement(
                            mt1.pose,
                            mt1.timestampSeconds);
                }
            }
        }
    }

    // private boolean inField(Pose2d pose) {
    // boolean inLength = pose.getX() > 0 && pose.getX() <
    // FieldConstants.FIELD_LENGTH;
    // boolean inWidth = pose.getY() > 0 && pose.getX() <
    // FieldConstants.FIELD_WIDTH;

    // return inLength && inWidth;
    // }
}
