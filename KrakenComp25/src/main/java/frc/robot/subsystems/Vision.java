// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.DirectMethodHandleDesc.Kind;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera camera = new PhotonCamera(getName());

  // Pose estimation
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Transform3d cameraToRobot = new Transform3d(new Translation3d(0, 12.25, 0), new Rotation3d());
  private Transform3d cameraToTarget;
  private Pose2d _pose;
  PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      cameraToRobot);

  // references from robot container
  RobotContainer rc = RobotContainer.getInstance();
  CommandSwerveDrivetrain driveTrain = rc.drivetrain;
  double forward = -rc.driver.getLeftY() * rc.MaxSpeed;
  double strafe = -rc.driver.getLeftY() * rc.MaxSpeed;
  double steer = -rc.driver.getLeftY() * rc.MaxAngularRate;

  public Vision() {
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    for (var change : camera.getAllUnreadResults()) {
        visionEst = poseEstimator.update(change);
    }
    return visionEst;
}


  public void UpdatePose() {
    var visionEst = getEstimatedGlobalPose();
    visionEst.ifPresent(est -> {
      driveTrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
      _pose = est.estimatedPose.toPose2d();
    });
  }

  // TODO: Find translation and rotation distances from april tags
  // TODO: Find how to drive request to position based off of april tags
  public void TargetReef() {
    boolean targetVisible = false;
    double targetRange = 0.0;
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        System.out.println("results found");

        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 1) {
            targetYaw = target.getYaw();
            targetRange = PhotonUtils.calculateDistanceToTargetMeters(targetYaw, targetYaw, targetRange, targetYaw);
          }
        }
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdatePose();
    Logger.recordOutput("VisionPose", _pose);
  }
}
