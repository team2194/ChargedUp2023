// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurnArm;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurnArmSubsystem;

public class JogTurnArm extends CommandBase {
  /** Creates a new JogArm. */
  private TurnArmSubsystem m_tas;
  private Supplier<Double> m_speed;

  public JogTurnArm(TurnArmSubsystem tas, Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_tas = tas;
    m_speed = speed;
    addRequirements(m_tas);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tas.jogArm(m_speed.get() / 2);
    m_tas.targetAngle = m_tas.getAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tas.jogArm(0);
    m_tas.targetAngle = m_tas.getAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
