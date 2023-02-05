// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grippers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GripperSubsystem;

public class RaiseGrippers extends CommandBase {
  /** Creates a new CloseGrippers. */
  private GripperSubsystem m_grip;
  private int lpctr;

  public RaiseGrippers(GripperSubsystem grip) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_grip = grip;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lpctr = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_grip.raiseGrippers();
    lpctr++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_grip.getGrippersRaised() && lpctr > 10;
  }
}
