package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LoadStationPickupConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.LimelightVision;

/**
 * Command to position accurately in front of any grid slot.
 * Robot needs to synch its position with an April Tag before using this
 * Command will drive to given X value and strafe to Y
 * If slot is for a pipe then LimeLight tape pipeline will
 * be selected and used to stop the robot on center.
 * Driver controls strafe speed until PIC controller takes over.
 * 
 */

public class DriveToLoadPickupPoint extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final GameHandlerSubsystem m_ghs;
  private final LimelightVision m_llv;
  private boolean m_blueAlliance;
  private boolean m_left;
  private DoubleSupplier m_strafeInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  private Pose2d endPose;
  private double endYTarget;
  private double endXTarget;

  private PIDController m_pidY = new PIDController(.5, 0, 0);

  private PIDController m_pidX = new PIDController(.5, 0, 0);

  private double drive_max = .25;

  public DriveToLoadPickupPoint(
      DriveSubsystem drive,
      GameHandlerSubsystem ghs,
      LimelightVision llv,
      boolean blueAlliance,
      boolean left,

      DoubleSupplier strafeInput) {

    m_drive = drive;
    m_ghs = ghs;
    m_llv = llv;
    m_blueAlliance = blueAlliance;
    m_left = left;
    m_strafeInput = strafeInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (m_blueAlliance) {
      endPose = LoadStationPickupConstants.blueLeftTarget;

    } else {
      endPose = LoadStationPickupConstants.blueRightTarget;
    }
    if (!m_blueAlliance) {
      endPose = LoadStationPickupConstants.redLeftTarget;

    } else {
      endPose = LoadStationPickupConstants.redRightTarget;
    }
    Pose2d currentPose = m_drive.getEstimatedPose();

    PathPlannerTrajectory trajLoad = PathPlanner.generatePath(

        new PathConstraints(2, 2),

        new PathPoint(currentPose.getTranslation(), currentPose.getRotation()),

        new PathPoint(endPose.getTranslation(), endPose.getRotation()));

  }

  /**
   * Used to position the robot to one of the load windows
   * Robot will drive normlly using gamepad until button held.
   * At that point if load april tag is seen it will create a Path Planner
   * trajectory. This will use endpoints from above.
   */
  // https://www.chiefdelphi.com/t/swerve-controller-joystick/392544/5
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4
    // m/s and max accel of 3 m/s^2

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopModules();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(endYTarget - m_drive.getY()) < .1;
  }
}
