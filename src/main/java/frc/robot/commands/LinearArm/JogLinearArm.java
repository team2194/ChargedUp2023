// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LinearArm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LinearArmSubsystem;

public class JogLinearArm extends CommandBase {
  /** Creates a new JogArm. */
  private LinearArmSubsystem m_las;
  private Supplier<Double> m_speed;

  public JogLinearArm(LinearArmSubsystem las, Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_las = las;
    m_speed = speed;
    addRequirements(m_las);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_las.jogLinearArm(m_speed.get() / 2);
    m_las.targetDistance = m_las.getDistance();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_las.jogLinearArm(0);
    m_las.targetDistance = m_las.getDistance();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
