// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LinearArm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurnArmSubsystem;

public class PositionLinearArm extends CommandBase {
  /** Creates a new PositionArm. */
  private TurnArmSubsystem m_tas;
  private double m_targetAngle;
  private int loopctr;

  public PositionLinearArm(TurnArmSubsystem tas, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tas = tas;
    m_targetAngle = targetAngle;
    addRequirements(m_tas);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopctr=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loopctr++;
    m_tas.goToPosition(m_targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return loopctr > 50 && m_tas.inPosition() && m_tas.isStopped();
  }
}
