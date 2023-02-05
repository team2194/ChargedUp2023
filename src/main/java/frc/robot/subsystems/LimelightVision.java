// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants2023;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  // private String limelightName = "limelight-tags";

  Rotation2d rr = new Rotation2d(1.57);
  Translation3d tl3 = new Translation3d(1, 2, 3);
  Rotation3d rr3 = new Rotation3d(1.57, 1.00, .44);
  public Transform3d tran3d = new Transform3d(tl3, rr3);

  public Pose2d visionPoseEstimatedData;

  public double imageCaptureTime;

  public int fiducialId;

  public Transform3d robotPose_FS;

  public boolean allianceBlue;

  public double distToTagX;
  public double distToTagY;
  public double degToTag;
  public double distToTag;

  public double visPosX;
  public double visPosY;
  public double visPosDeg;

  public double robX;
  public double robY;
  public double robDeg;

  public double robDiffX;
  public double robDiffY;
  public double robDiffDeg;

  public double curTagX;

  public double curTagY;

  public double curTagDeg;
  private int currentPipelineIndex;
  private double[] CurrentRobotPose_FS;

  static enum pipelinetype {
    retroreflective,
    grip,
    python,
    fiducialmarkers,
    classifier,
    detector
  }

  static enum pipelines {
    TAPE0(0, pipelinetype.retroreflective),
    LOADCUBE(1, pipelinetype.fiducialmarkers),
    LOADCONE(2, pipelinetype.fiducialmarkers),
    SPARE3(3, pipelinetype.fiducialmarkers),
    TAGS(4, pipelinetype.fiducialmarkers),
    SPARE5(5, pipelinetype.fiducialmarkers),
    TAPE(6, pipelinetype.retroreflective),
    SPARE7(7, pipelinetype.fiducialmarkers),
    SPARE8(8, pipelinetype.detector),
    SPARE9(9, pipelinetype.classifier);

    private int number;

    private pipelinetype type;

    private pipelines(int number, pipelinetype type) {
      this.number = number;
      this.type = type;
    }

    private int getNumber() {
      return number;
    }

    private pipelinetype getType() {
      return type;
    }
  }

  public LimelightVision() {

  }

  public void setAllianceBlue(boolean alliance) {
    allianceBlue = alliance;
  }

  @Override
  public void periodic() {

    LimelightHelpers.getBotPose_TargetSpace("");
    // LimelightResults llResults = LimelightHelpers.getLatestResults("");

    fiducialId = (int) LimelightHelpers.getFiducialID("");
    currentPipelineIndex = (int) LimelightHelpers.getCurrentPipelineIndex("");
   
  }

  

  public double round2dp(double number) {
    number = Math.round(number * 100);
    number /= 100;
    return number;
  }

  public void setLoadCubePipeline() {

    LimelightHelpers.setPipelineIndex("", pipelines.LOADCUBE.ordinal());

  }

  public void setLoadConePipeline() {
    LimelightHelpers.setPipelineIndex("", pipelines.LOADCONE.ordinal());
  }

  public void setTapePipeline() {
    LimelightHelpers.setPipelineIndex("", pipelines.LOADCONE.ordinal());
  }

  public void setAprilTagPipeline() {
    LimelightHelpers.setPipelineIndex("", pipelines.LOADCONE.ordinal());
  }

  public Pose3d getTagPose(int tagID) {
    return FieldConstants2023.aprilTags.get(tagID);
  }

  public Pose3d getCurrentTagPose() {
    return FieldConstants2023.aprilTags.get(currentPipelineIndex);
  }

}
