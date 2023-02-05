package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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


public class StrafeToSlot extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_drive;
  private final GameHandlerSubsystem m_ghs;
  private final LimelightVision m_llv;
  private DoubleSupplier m_strafeInput;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerveDriveSubsystem The subsystem used by this command.
   */
  private double endYTarget;
  private double endXTarget;
  private boolean isPipe;

  private PIDController m_pidY = new PIDController(.5, 0, 0);


  private PIDController m_pidX = new PIDController(.5, 0, 0);
  private double drive_max = .25;

  private double ledsOnDist;

  public StrafeToSlot(
      DriveSubsystem drive,
      GameHandlerSubsystem ghs,
      LimelightVision llv,

      DoubleSupplier strafeInput) {

    m_drive = drive;
    m_ghs = ghs;
    m_llv = llv;
    m_strafeInput = strafeInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_llv.setAprilTagPipeline();

    endYTarget = m_ghs.getActiveDrop().getYVal();
    endXTarget = m_ghs.getActiveDrop().getXVal();
    isPipe = m_ghs.getActiveDrop().getIsPipe();

    SmartDashboard.putNumber("ENDPTY", endYTarget);
    SmartDashboard.putNumber("ENDPTX", endXTarget);
    SmartDashboard.putBoolean("PIPE", isPipe);

  }

  /**
   * Used to position the robot to one of the target slots
   * Robot will run at 50% strafe until it reaches the slowdown distance
   * from the target end.
   * T that point speed will be reduced proportional to remaining distance
   * 
   * 
   */
  // https://www.chiefdelphi.com/t/swerve-controller-joystick/392544/5
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidOutY = 0;
    boolean targetDirectionIsPlusY = endYTarget > m_drive.getY();

    boolean operatorDirectionIsPlusY = m_strafeInput.getAsDouble() > 0;

    if (targetDirectionIsPlusY != operatorDirectionIsPlusY) {

      pidOutY = m_strafeInput.getAsDouble();

    }

    else {

      pidOutY = m_pidY.calculate(m_drive.getY(), endYTarget);

      if (Math.abs(pidOutY) > Math.abs(m_strafeInput.getAsDouble())) {

        pidOutY = m_strafeInput.getAsDouble();

        if (!targetDirectionIsPlusY)

          pidOutY = -pidOutY;
      }
    }

    double pidOutX = m_pidX.calculate(m_drive.getX(), endYTarget);

    double latchpidX = pidOutX;

    if (Math.abs(pidOutX) > drive_max) {

      pidOutX = drive_max;

      if (latchpidX < 0)

        pidOutX = -pidOutX;
    }

    m_drive.drive(pidOutX, pidOutY, 0);

    if (isPipe && Math.abs(endYTarget - m_drive.getY()) < ledsOnDist)

      m_llv.setTapePipeline();

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
