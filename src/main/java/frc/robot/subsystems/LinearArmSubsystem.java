// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearArmSubsystem extends SubsystemBase {
  /** Creates a new LinearArmSubsystem. */
  private static final int kMotorPort = 1;
  private static final int kEncoderAChannel = 2;
  private static final int kEncoderBChannel = 3;

  private static final double kLinearArmKp = .1;
  private static final double kLinearArmGearing = 10.0;
  private static final double kLinearArmDrumRadius = Units.inchesToMeters(2.0);
  private static final double kCarriageMass = 4.0; // kg

  private static final double minLinearArmDistance = Units.inchesToMeters(2);

  private static final double maxLinearArmDistance = Units.inchesToMeters(50);

  public enum linearDistances {
    MIN_DIST(Units.inchesToMeters(2)),
    FLOOR_PICKUP(Units.inchesToMeters(20)),
    FLOOR_DROPOFF(Units.inchesToMeters(25)),
    MID_DROPOFF(-10),
    HIGH_DROPOFF(20),
    LOAD_PICKUP(10);

    private final double distance;

    private linearDistances(double distance) {
      this.distance = distance;
    }

    public double getDistance() {
      return distance;
    }

  }

  // distance per pulse = (distance per revolution) / (pulses per revolution)
  // = (Pi * D) / ppr
  private static final double kLinearArmEncoderDistPerPulse = 2.0 * Math.PI * kLinearArmDrumRadius / 4096;

  private final DCMotor m_LinearArmGearbox = DCMotor.getVex775Pro(4);

  // Standard classes for controlling our LinearArm
  private final PIDController m_controller = new PIDController(kLinearArmKp, 0, 0);

  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);

  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);

  private final PWMSparkMax m_wristMotor = new PWMSparkMax(3);

  private final AnalogPotentiometer m_wristPot = new AnalogPotentiometer(1, 90);
  // Simulation classes help us simulate what's going on, including gravity.

  private final ElevatorSim m_LinearArmSim = new ElevatorSim(
      m_LinearArmGearbox,
      kLinearArmGearing,
      kCarriageMass,
      kLinearArmDrumRadius,
      minLinearArmDistance,
      maxLinearArmDistance,
      false,
      VecBuilder.fill(0.01));
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d visualization of the LinearArm
  private final Mechanism2d m_mech2d = new Mechanism2d(20, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("LinearArm Root", 10, 010);
  private final MechanismLigament2d m_LinearArmMech2d = m_mech2dRoot.append(
      new MechanismLigament2d(
          "LinearArm", Units.metersToInches(m_LinearArmSim.getPositionMeters()), 00));
  private MechanismLigament2d m_wrist;

  public double targetDistance;
  private double activeTargetDistance;
  private boolean m_testMode;

  public LinearArmSubsystem() {

    m_wrist = m_LinearArmMech2d.append(
        new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    m_encoder.setDistancePerPulse(kLinearArmEncoderDistPerPulse);

    SmartDashboard.putData("LinArm Sim", m_mech2d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LinArmdistance", getDistance());
    SmartDashboard.putNumber("LinArmtarget", targetDistance);
    SmartDashboard.putNumber("LinArmVel", getVelocity());

    SmartDashboard.putBoolean("LinInPos", inPosition());
    SmartDashboard.putBoolean("LinStopped", isStopped());

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our LinearArm is doing
    // First, we set our "inputs" (voltages)
    m_LinearArmSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // // Next, we update it. The standard loop time is 20ms.
    m_LinearArmSim.update(0.020);
    m_wrist.setAngle(m_wristPot.get());
    // Finally, we set our simulated encoder's readings and simulated battery
    // voltage
    m_encoderSim.setDistance(m_LinearArmSim.getPositionMeters());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_LinearArmSim.getCurrentDrawAmps()));

    // Update LinearArm visualization with simulated position
    m_LinearArmMech2d.setLength(Units.metersToInches(m_LinearArmSim.getPositionMeters()));
  }

  public void jogLinearArm(double speed) {

    m_motor.setVoltage(speed * RobotController.getBatteryVoltage());

  }

  public void goToPosition(double distance) {

    double pidOut = m_controller.calculate(getDistance(), distance);

    m_motor.setVoltage(pidOut * RobotController.getBatteryVoltage());

    targetDistance = getDistance();
  }

  public void positionHold() {
    goToPosition(targetDistance);
  }

  public double getDistance() {
    return Units.radiansToDegrees(m_encoder.getDistance());
  }

  public double getVelocity() {
    return m_encoder.getRate();
  }

  public boolean inPosition() {
    return Math.abs(targetDistance - getDistance()) < .25;
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < .1;
  }

  public void setActiveTargetDistance(double Distance) {
    activeTargetDistance = Distance;
  }

  public double getActiveTargetDistance() {
    return activeTargetDistance;
  }

  public void setTestMode(){
    m_testMode=true;
  }

}
