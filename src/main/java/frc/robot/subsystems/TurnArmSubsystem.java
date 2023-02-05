// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurnArmSubsystem extends SubsystemBase {
  /** Creates a new DualArmSubsystem. */
  private static final int kMotorPort = 0;
  private static final int kEncoderAChannel = 0;
  private static final int kEncoderBChannel = 1;
  private static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  // The P gain for the PID controller that drives this arm.
  private static double kArmKp = .50;

  public double minArmPositionDeg = -45.0;
  public static double maxArmPositionDeg = 50.0;

  public enum turnAngles {
    MIN_ANGLE(-15),
    FLOOR_PICKUP(-35),
    FLOOR_DROPOFF(-30),
    MID_DROPOFF(-10),
    HIGH_DROPOFF(20),
    MAX_ANGLE(35);


    private final double angle;

    private turnAngles(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }

  }

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  // = (2 * PI rads) / (4096 pulses)
  private static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;// rads per motor rev

  // The arm gearbox represents a gearbox containing two Vex 775pro motors.
  private final DCMotor m_turnArmGearbox = DCMotor.getVex775Pro(2);

  // Standard classes for controlling our arm
  private final PIDController m_controller = new PIDController(kArmKp, 0, 0);
  private final Encoder m_encoder = new Encoder(kEncoderAChannel, kEncoderBChannel);
  private final PWMSparkMax m_motor = new PWMSparkMax(kMotorPort);
  public double targetAngle;
  // Simulation classes help us simulate what's going on, including gravity.
  private static final double m_turnArmReduction = 600;
  private static final double m_turnArmMass = 5.0; // Kilograms
  private static final double m_turnArmLength = Units.inchesToMeters(30);
  private double armTravel = maxArmPositionDeg - minArmPositionDeg;
  private double motorRevsForTravel = m_turnArmReduction;
  private double degreesPerMotorRev = armTravel / motorRevsForTravel;
  private double degreesPerCount = degreesPerMotorRev;

  // This arm sim represents an arm that can travel from -45 degrees (rotated down
  // front)
  // to 45 degrees (rotated up in front).
  private final SingleJointedArmSim m_turnArmSim = new SingleJointedArmSim(
      m_turnArmGearbox,
      m_turnArmReduction,
      SingleJointedArmSim.estimateMOI(m_turnArmLength, m_turnArmMass),
      m_turnArmLength,
      Units.degreesToRadians(minArmPositionDeg),
      Units.degreesToRadians(maxArmPositionDeg),
      m_turnArmMass,
      true,
      VecBuilder.fill(kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_turnArmPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_turnArmTower = m_turnArmPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_turnArm = m_turnArmPivot.append(
      new MechanismLigament2d(
          "TurnArm",
          30,
          Units.radiansToDegrees(m_turnArmSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow)));
  private double activeTargetAngle;
  private boolean m_testMode;

  public TurnArmSubsystem() {

    m_encoder.setDistancePerPulse(kArmEncoderDistPerPulse);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Turn Arm Sim", m_mech2d);
    m_turnArmTower.setColor(new Color8Bit(Color.kBlue));
    targetAngle = minArmPositionDeg;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("TurnArmAngle", getAngle());
    SmartDashboard.putNumber("TurnTargetAngle", targetAngle);
    SmartDashboard.putNumber("TurnArmVel", getVelocity());

    SmartDashboard.putBoolean("TurnInPos", inPosition());
    SmartDashboard.putBoolean("TurnStopped", isStopped());

  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // // First, we set our "inputs" (voltages)
    // m_turnArmSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // // Next, we update it. The standard loop time is 20ms.
    // m_turnArmSim.update(0.020);

    // // Finally, we set our simulated encoder's readings and simulated battery
    // // voltage
    // m_encoderSim.setDistance(m_turnArmSim.getAngleRads());
    // // SimBattery estimates loaded battery voltages
    // RoboRioSim.setVInVoltage(
    //     BatterySim.calculateDefaultBatteryLoadedVoltage(m_turnArmSim.getCurrentDrawAmps()));

    // // Update the Mechanism Arm angle based on the simulated arm angle
    // m_turnArm.setAngle(Units.radiansToDegrees(m_turnArmSim.getAngleRads()));
  }

  public void jogArm(double speed) {

    m_motor.setVoltage(speed * RobotController.getBatteryVoltage());

  }

  public void goToPosition(double degrees) {

    double pidOut = m_controller.calculate(getAngle(), degrees);

    m_motor.setVoltage(pidOut * RobotController.getBatteryVoltage());

    targetAngle = getAngle();
  }

  public void positionHold() {
    goToPosition(targetAngle);
  }

  public double getAngleRads() {
    return m_encoder.getDistance();
  }

  public double getAngle() {
    return Units.radiansToDegrees(m_encoder.getDistance());
  }

  public double getVelocity() {
    return m_encoder.getRate();
  }

  public boolean inPosition() {
    return Math.abs(targetAngle - getAngle()) < .25;
  }

  public boolean isStopped() {
    return Math.abs(getVelocity()) < .1;
  }

  public void setActiveTargetAngle(double angle) {
    activeTargetAngle = angle;
  }

  public double getActiveTargetAngle() {
    return activeTargetAngle;
  }

  public void setTestMode(){
    m_testMode=true;
  }
}
