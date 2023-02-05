// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SolenoidConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */

  private DoubleSolenoid m_gripperOpenClose

      = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
          SolenoidConstants.GRIPPER_OPEN,
          SolenoidConstants.GRIPPER_CLOSE);

  private DoubleSolenoid m_gripperRaiseLower

      = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
          SolenoidConstants.GRIPPER_RAISE,
          SolenoidConstants.GRIPPER_LOWER);

  private boolean m_testMode;

  public GripperSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void raiseGrippers() {
    m_gripperRaiseLower.set(DoubleSolenoid.Value.kForward);
  }

  public void lowerGrippers() {
    m_gripperRaiseLower.set(DoubleSolenoid.Value.kReverse);
  }

  public void openGrippers() {
    m_gripperOpenClose.set(DoubleSolenoid.Value.kForward);
  }

  public void closeGrippers() {
    m_gripperOpenClose.set(DoubleSolenoid.Value.kReverse);
  }

  public boolean getGrippersRaised() {
    return m_gripperRaiseLower.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getGrippersLowered() {
    return m_gripperRaiseLower.get() == DoubleSolenoid.Value.kReverse;
  }

  public boolean getGrippersOpen() {
    return m_gripperOpenClose.get() == DoubleSolenoid.Value.kForward;
  }

  public boolean getGrippersClosed() {
    return m_gripperOpenClose.get() == DoubleSolenoid.Value.kReverse;
  }

  public void setTestMode() {
    m_testMode = true;
  }

}
