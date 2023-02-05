// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurnArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurnArmSubsystem;

public class PositionHoldTurnArm extends CommandBase {
  /** Creates a new PositionHoldArm. */
  private TurnArmSubsystem m_tas;

  public PositionHoldTurnArm(TurnArmSubsystem tas) {
    m_tas = tas;
    addRequirements(m_tas);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tas.positionHold();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
