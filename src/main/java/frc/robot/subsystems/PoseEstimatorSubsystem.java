package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
//import frc.robot.subsystems.vision.Camera;
import frc.robot.vision.LimelightHelpers;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private DriveSubsystem driveSubsystem;
  
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

  private SwerveDrivePoseEstimator poseEstimator;

Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

// Construct PhotonPoseEstimator
PhotonPoseEstimator photonPoseEstimator;

  public PhotonCamera camera;

  private final Field2d field2d = new Field2d();
  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public PoseEstimatorSubsystem(){}
  
  public void init(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;

    
    camera = new PhotonCamera("PhotonCamera"); 
    photonPoseEstimator  = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    
    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        driveSubsystem.getGyroscopeRotation(),
        driveSubsystem.getModulePositions(),
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);

  }

  void resetPosition(Rotation2d rotation, Pose2d pose ) {
    poseEstimator.resetPosition(rotation, driveSubsystem.getModulePositions(), pose);
  }

  @Override
  public void periodic() {

    if(driveSubsystem == null)
      return;

      


    // Update pose estimator with driveSubsystem sensors
    poseEstimator.update(
      driveSubsystem.getGyroscopeRotation(),
      driveSubsystem.getModulePositions());

    updateVision();

    field2d.setRobotPose(getCurrentPose());
  }

  private String getFomattedPose() {
    return getFomattedPose(getCurrentPose());
  }

  private String getFomattedPose(Pose2d pose) {

    return String.format("(%.2f, %.2f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
      driveSubsystem.getGyroscopeRotation(),
      driveSubsystem.getModulePositions(),
      newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public boolean blueAlliance() {
      return DriverStation.getAlliance().get() == Alliance.Blue;
  }


  private Pose2d previousPose;
  public void updatePhotonVision() {
    if(previousPose != null) {
      photonPoseEstimator.setReferencePose(previousPose);
  }
  for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
    Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
    if(estimatedPose.isPresent()) {
      previousPose = estimatedPose.get().estimatedPose.toPose2d();
      PhotonTrackedTarget target = result.getBestTarget();
      Pose2d pose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), estimatedPose.get().estimatedPose, robotToCam.inverse()).toPose2d();

      poseEstimator.addVisionMeasurement(pose ,estimatedPose.get().timestampSeconds);
    }  
  }
    
}


  public void updateVision() {
    boolean useMegaTag2 = false; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-one");
      
      if (mt1 != null ) {
        if( mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
        {
          if(mt1.rawFiducials[0].ambiguity > .7)
          {
            doRejectUpdate = true;
          }
          if(mt1.rawFiducials[0].distToCamera > 3)
          {
            doRejectUpdate = true;
          }
        }
        if(mt1.tagCount == 0)
        {
          doRejectUpdate = true;
        }

        if(!doRejectUpdate)
        {
          SmartDashboard.putNumber("Angle", mt1.pose.getRotation().getDegrees());
          poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
          poseEstimator.addVisionMeasurement(
              mt1.pose,
              mt1.timestampSeconds);

          SmartDashboard.putString("VisionTag", getFomattedPose(mt1.pose));

        }
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight-one", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one");
      if(Math.abs(driveSubsystem.getTurnRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }
}