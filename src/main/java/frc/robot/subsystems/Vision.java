// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
  public static class Constants {
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> kStateSTDdevs = VecBuilder.fill(0.1,0.1,0.1);

    static Translation3d cameraTranslation = new Translation3d(0.0, 0, 0.25);
    static Rotation3d cameraRotation = new Rotation3d(0, Math.toRadians(0), 0);
    static Transform3d cameraPos = new Transform3d(cameraTranslation, cameraRotation);
  }

  Pose2d estimatedPose = new Pose2d();
  VisionSystemSim visionSim = new VisionSystemSim("visionSim");
  SwerveDrivePoseEstimator estimator;
  SwerveBase swerve;
  Pose2d startPose;
  double lastEstTimestamp;
  PhotonCameraSim cameraSim;
  
  StructPublisher<Pose3d> cameraPose;
  StructPublisher<Pose2d> fusedPose;

  PhotonCamera camera = new PhotonCamera("cameraName");



  PhotonPoseEstimator photonPoseEst = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.cameraPos);

  /** Creates a new VisionSim. */
  public Vision(SwerveBase swerve) {
    this.swerve = swerve;
    
    startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    cameraPose = NetworkTableInstance.getDefault().getStructTopic("Camera Pose", Pose3d.struct).publish();
    fusedPose = NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    if(Robot.isSimulation()){
      
      
      AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
      visionSim.addAprilTags(tags);

      SimCameraProperties cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(960,720, Rotation2d.fromDegrees(82.389));
      cameraProperties.setFPS(20);
      cameraProperties.setAvgLatencyMs(35);
      cameraProperties.setLatencyStdDevMs(5);

      cameraSim = new PhotonCameraSim(camera, cameraProperties);

      visionSim.addCamera(cameraSim, Constants.cameraPos);
    }

    estimator = new SwerveDrivePoseEstimator(Swerve.Constants.kinematics, swerve.getYaw(), swerve.getModulePositions(), startPose, Constants.kStateSTDdevs, Constants.kSingleTagStdDevs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    estimator.updateWithTime(Timer.getFPGATimestamp(), swerve.getYaw(), swerve.getModulePositions());
    estimatedPose = estimator.getEstimatedPosition();

    fusedPose.set(estimatedPose);
    if(Robot.isSimulation()){
      visionSim.update(estimatedPose);
      //So Camera View is visible in Ascope
      // cameraPose.set(visionSim.getCameraPose(cameraSim).get());
      //Math Not working why
      cameraPose.set(new Pose3d(Constants.cameraTranslation.rotateBy(fieldRot3d()).plus(fieldTrans3d()), Constants.cameraRotation.plus(fieldRot3d())));//fieldRot3d()));
    } else{
      cameraPose.set(new Pose3d(Constants.cameraTranslation.rotateBy(fieldRot3d()).plus(fieldTrans3d()), Constants.cameraRotation.plus(fieldRot3d())));
    }

    getEstimatedGlobalPose(estimatedPose).ifPresent(this::addVisionMeasurement);
   
  }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEst.setReferencePose(prevEstimatedRobotPose);
        var visionEst = photonPoseEst.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est ->
                            getSimDebugField()
                                    .getObject("VisionEstimation")
                                    .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    public Field2d getSimDebugField(){
      if(Robot.isReal())return null;
      return visionSim.getDebugField();
    }

        public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = Constants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonPoseEst.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = Constants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public PhotonPipelineResult getLatestResult() {
      return camera.getLatestResult();
    }

    public Translation3d fieldTrans3d(){
      Translation2d est = estimatedPose.getTranslation();
      return new Translation3d(est.getX(),est.getY(),0);
    }

    public Rotation3d fieldRot3d(){
      Rotation2d est = estimatedPose.getRotation();
      return new Rotation3d(0, 0, est.getRadians());
    }

    public void addVisionMeasurement(EstimatedRobotPose estRobotPose){
       estimator.addVisionMeasurement(estRobotPose.estimatedPose.toPose2d(), estRobotPose.timestampSeconds, getEstimationStdDevs(estRobotPose.estimatedPose.toPose2d()));
    }

    public Pose2d getRobotPose(){
      return estimatedPose;
    }
}
