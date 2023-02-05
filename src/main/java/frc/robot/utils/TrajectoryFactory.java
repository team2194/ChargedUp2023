// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldConstants2023;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class TrajectoryFactory {

    public SendableChooser<String> ppTrajChooser = new SendableChooser<String>();

    public SendableChooser<Integer> tagTrajChooser = new SendableChooser<Integer>();

    private DriveSubsystem m_drive;

    private boolean tune;

    public boolean run;

    public boolean runTagTraj;

    private String pathOne = "PathOne";
    private String pathTwo = "PathOne";
    private String pathThree = "PathOne";
    private String pathFour = "PathOne";
    private String pathFive = "PathOne";

    public TrajectoryFactory(DriveSubsystem drive) {

        m_drive = drive;
        ppTrajChooser.setDefaultOption("DriveForward", "DriveForward");
        ppTrajChooser.addOption(pathOne, pathOne);
        ppTrajChooser.addOption(pathTwo, pathTwo);
        ppTrajChooser.addOption(pathThree, pathThree);
        ppTrajChooser.addOption(pathFour, pathFour);
        ppTrajChooser.addOption(pathFive, pathFive);

        SmartDashboard.putData("TrajChoice", ppTrajChooser);

        tagTrajChooser.setDefaultOption("Tag 3", 3);
        tagTrajChooser.addOption("Tag Four", 4);
        tagTrajChooser.addOption("Tag Five", 5);
        tagTrajChooser.addOption("Tag Six", 6);
        SmartDashboard.putData("TagTrajChoice", tagTrajChooser);

    }

    public String getSelectedTrajectory() {
        return ppTrajChooser.getSelected();
    }

    public PathPlannerTrajectory getTrajectory(String name) {
        return PathPlanner.loadPath(name, 1, 1, false);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        m_drive.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(

                        traj,

                        m_drive::getEstimatedPose, // Pose supplier

                        m_drive.m_kinematics, // SwerveDriveKinematics

                        m_drive.getXPID(),

                        m_drive.getYPID(),

                        m_drive.getThetaPID(),

                        m_drive::setModuleStates, // Module states consumer

                        m_drive // Requires this drive subsystem
                ),

                new InstantCommand(() -> m_drive.stopModules()),

                new ParallelCommandGroup(

                        new InstantCommand(() -> run = false),

                        new InstantCommand(() -> runTagTraj = false)));
    }

    public void setRun(boolean on) {
        run = on;
    }

    public void periodic() {

        if (run) {
            PathPlannerTrajectory trajectory1 = PathPlanner.loadPath(
                    getSelectedTrajectory(), 2, 2, false);
            SmartDashboard.putString("Path", getSelectedTrajectory());
            this.followTrajectoryCommand(trajectory1, true).schedule();
            run = false;
        }
        if (runTagTraj) {
            PathPlannerTrajectory trajectory1 = moveToTag(tagTrajChooser.getSelected(), 2, 1);
            this.followTrajectoryCommand(trajectory1, true).schedule();
            runTagTraj = false;
        }

    }

    public PathPlannerTrajectory moveToTag(int id, double maxvel, double maxAccel) {

        Pose2d tagPose = FieldConstants2023.aprilTags.get(id).toPose2d();
        Pose2d robotPose = m_drive.getEstimatedPose();

        PathPlannerTrajectory trajt = PathPlanner.generatePath(

                new PathConstraints(maxvel, maxAccel),

                new PathPoint(robotPose.getTranslation(), robotPose.getRotation()), // position, heading

                new PathPoint(tagPose.getTranslation(), tagPose.getRotation()) // position, heading

        );
        return trajt;
    }

    public PathPlannerTrajectory getSimpleTraj() {

        // Simple path without holonomic rotation. Stationary start/end. Max velocity of
        // 4 m/s and max accel of 3 m/s^2

        PathPlannerTrajectory traj1 = PathPlanner.generatePath(

                new PathConstraints(2, 2),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
        );
        return traj1;
    }

    public PathPlannerTrajectory getSimpleTrajWithHolonomic() {

        // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4
        // m/s and max accel of 3 m/s^2
        PathPlannerTrajectory traj2 = PathPlanner.generatePath(
                new PathConstraints(4, 3),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position,

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)) // position,

        );

        return traj2;
    }

    // More complex path with holonomic rotation. Non-zero starting velocity of 2
    // m/s. Max velocity of 4 m/s and max accel of 3 m/s^2

    public PathPlannerTrajectory getNonZeroStartTraj() {

        PathPlannerTrajectory traj3 = PathPlanner.generatePath(

                new PathConstraints(4, 3),

                new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0), 2), // position,

                new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(-90)), // position,

                new PathPoint(new Translation2d(5.0, 3.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-30)) // position,

        );
        return traj3;
    }

}
