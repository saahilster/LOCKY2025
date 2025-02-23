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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final PhotonCamera camera = new PhotonCamera("Vision");
  // private final PhotonCamera cameraClimb = new PhotonCamera("Climb");
  // Pose estimation
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private final Transform3d cameraToRobot = new Transform3d(new Translation3d(0, 12.25, 0), new Rotation3d());
  private Transform3d cameraToTarget;
  private Pose2d _pose;
  PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      cameraToRobot);

  //in degrees
  double cameraPitch = 20;
  //in inches
  double cameraHeight = 5;

  boolean targetVisible = false;
  double targetRange = 0.0;
  double targetYaw = 0.0;

  // references from robot container
  RobotContainer rc = RobotContainer.getInstance();
  CommandSwerveDrivetrain driveTrain = rc.drivetrain;

  //Singleton
  private static Vision instance;

  public static Vision getInstance(){
    if(instance == null){
      instance = new Vision();
    }
    return instance;
  }

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
      double timestamp = est.timestampSeconds;
      driveTrain.addVisionMeasurement(driveTrain.getState().Pose, timestamp);
      System.out.println("Vision Pose: " + est.estimatedPose.toPose2d());
    });
  }

  // TODO: Find translation and rotation distances from april tags
  // TODO: Find how to drive request to position based off of april tags
  public void GetReefTarget() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        System.out.println("results found");

        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 6 ||
              target.getFiducialId() == 7 ||
              target.getFiducialId() == 8 ||
              target.getFiducialId() == 9 ||
              target.getFiducialId() == 10 ||
              target.getFiducialId() == 11) {

            targetYaw = target.getYaw();
            targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                Units.feetToMeters(cameraHeight),
                Units.inchesToMeters(8.75),
                Units.degreesToRadians(cameraPitch),
                Units.degreesToRadians(target.getPitch()));
            targetVisible = true;
          }
        }
      }
      else{
        targetVisible = false;
      }
    }
  }

  public Command RequestReef(){
    Command pathFindingCommand = null;

    var waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d());

    PathPlannerPath path = new PathPlannerPath(
      null, 
      null, 
      null, 
      null);

    pathFindingCommand = AutoBuilder.followPath(path);

    return pathFindingCommand;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    UpdatePose();
    Logger.recordOutput("VisionPose", driveTrain.getState().Pose);
    System.out.println("Vision Pose: " + driveTrain.getState().Pose);
    
  }
}
