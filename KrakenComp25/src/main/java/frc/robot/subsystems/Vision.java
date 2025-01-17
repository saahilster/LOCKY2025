// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.DirectMethodHandleDesc.Kind;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera camera = new PhotonCamera(getName());
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Transform3d cameraToRobot = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d());
  private Pose3d _pose;
  
  public Vision() {}

  public Pose3d UpdatePose(){
    var result = camera.getAllUnreadResults();
    Pose3d currentPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToRobot, _pose, cameraToRobot);
    return currentPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
