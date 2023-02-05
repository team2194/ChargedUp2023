// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LinearArm.JogLinearArm;
import frc.robot.commands.LinearArm.PositionHoldLinearArm;
import frc.robot.commands.TurnArm.JogTurnArm;
import frc.robot.commands.TurnArm.PositionHoldTurnArm;


import frc.robot.commands.swerve.RotateToAngle;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.swerve.StrafeToSlot;

import frc.robot.oi.CommandLeonardoController;
import frc.robot.oi.CommandPS3Controller;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameHandlerSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.LinearArmSubsystem;
import frc.robot.subsystems.TurnArmSubsystem;
import frc.robot.utils.AutoFactory;
import frc.robot.utils.LEDControllerI2C;
import frc.robot.utils.PoseTelemetry;
import frc.robot.utils.TrajectoryFactory;

public class RobotContainer {

        // The robot's subsystems
        final DriveSubsystem m_drive;

        final TurnArmSubsystem m_turnArm;// = new TurnArmSubsystem();

        final LinearArmSubsystem m_linArm;// = new LinearArmSubsystem();

        final GripperSubsystem m_grp;

        final LimelightVision m_llv = new LimelightVision();

        public AutoFactory m_autoFactory;

        public TrajectoryFactory m_tf;

        public GameHandlerSubsystem m_ghs;

        public LEDControllerI2C lcI2;

        public final FieldSim m_fieldSim;

        // The driver and codriver controllers

        private CommandPS4Controller m_driverController = new CommandPS4Controller(
                        OIConstants.kDriverControllerPort);

        private CommandPS3Controller m_coDriverController = new CommandPS3Controller(
                        OIConstants.kCoDriverControllerPort);

        public CommandLeonardoController m_codriverBox = new CommandLeonardoController(5);

        public PoseTelemetry pt = new PoseTelemetry();

        final PowerDistribution m_pdp = new PowerDistribution();

        public LimelightVision m_llvis = new LimelightVision();


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                Pref.deleteUnused();

                Pref.addMissing();

                m_drive = new DriveSubsystem();

                m_drive.showOnShuffleboard = false;

                m_turnArm = new TurnArmSubsystem();

                m_linArm = new LinearArmSubsystem();

                m_grp = new GripperSubsystem();

                SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

                LiveWindow.disableAllTelemetry();

                m_autoFactory = new AutoFactory(m_drive, m_turnArm, m_linArm);

                m_tf = new TrajectoryFactory(m_drive);

                m_ghs = new GameHandlerSubsystem();

                m_fieldSim = new FieldSim(m_drive);

        
                // m_ls = new LightStrip(9, 60);

                // lc = LEDController.getInstance();

                // lcI2 = LEDControllerI2C.getInstance()

                SmartDashboard.putData("Drive", m_drive);
                SmartDashboard.putData("TurnArm", m_turnArm);
                SmartDashboard.putData("Lin Arm", m_linArm);
                SmartDashboard.putData("Grippers", m_ghs);

                // PortForwarder.add(5800, "10.21.94.11", 5800);
                // PortForwarder.add(1181, "10.21.94.11", 1181);
                // PortForwarder.add(1182, "10.21.94.11", 1182);
                // PortForwarder.add(1183, "10.21.94,11", 1183);
                // PortForwarder.add(1184, "10.21.94.11", 1184);

                CommandScheduler.getInstance()
                                .onCommandInitialize(command -> System.out.println(command.getName() + " is starting"));
                CommandScheduler.getInstance()
                                .onCommandFinish(command -> System.out.println(command.getName() + " has ended"));
                CommandScheduler.getInstance()
                                .onCommandInterrupt(
                                                command -> System.out.println(command.getName() + " was interrupted"));
                CommandScheduler.getInstance().onCommandInitialize(
                                command -> SmartDashboard.putString("CS", command.getName() + " is starting"));
                CommandScheduler.getInstance()
                                .onCommandFinish(command -> SmartDashboard.putString("CE",
                                                command.getName() + " has Ended"));
                CommandScheduler.getInstance().onCommandInterrupt(
                                command -> SmartDashboard.putString("CE", command.getName() + "was Interrupted"));

                m_fieldSim.initSim();

                setDefaultCommands();

                configDriverButtons();

                configCodriverButtons();

                configLeonardoBoxButtons();


        }

        private void setDefaultCommands() {

                m_drive.setDefaultCommand(getDriveCommand());

                m_linArm.setDefaultCommand(new PositionHoldLinearArm(m_linArm));

                m_turnArm.setDefaultCommand(new PositionHoldTurnArm(m_turnArm));

        }

        void configDriverButtons() {

                m_driverController.square().whileTrue(getStrafeToTargetCommand());

                m_driverController.povLeft().onTrue(new RotateToAngle(m_drive, 90));

                m_driverController.povDown().onTrue(new InstantCommand(() -> m_ghs.setDropOffLevel(0)));

                m_driverController.povRight().onTrue(new InstantCommand(() -> m_ghs.setDropOffLevel(1)));

                m_driverController.povUp().onTrue(new InstantCommand(() -> m_ghs.setDropOffLevel(2)));

                m_driverController.cross().onTrue(new InstantCommand(() -> m_ghs.setConeForPickup()))
                                .onTrue(new InstantCommand(() -> m_llv.setLoadConePipeline()));

                m_driverController.circle().onTrue(new InstantCommand(() -> m_ghs.setCubeForPickup()))
                                .onTrue(new InstantCommand(() -> m_llv.setLoadCubePipeline()));

               // m_driverController.L2().onTrue(getDriveToTapeCommand());

                // m_driverController.R2()

                // m_driverController.triangle()

        }

        void configDriverButtons(boolean bluelliance) {

                int tag = 4;

                if (!bluelliance)

                        tag = 5;

                // m_driverController.L1()
                //                 .onTrue(new ChaseTagCommandLimelight(m_llvis, tag, 1, 0, 0, m_drive,
                //                                 () -> -m_driverController.getLeftX()));

                // m_driverController.L2()
                //                 .onTrue(new ChaseTagCommandLimelight(m_llvis, m_llv tag, -1, 0, 0, m_drive,
                //                                 () -> -m_driverController.getLeftX()));

        }

        private void configCodriverButtons() {

                m_coDriverController.R1()
                                .onTrue(Commands.runOnce(() -> m_tf.setRun(true)));
                ;

                m_coDriverController.L1()
                                .onTrue(getJogLinearArmCommand());

                m_coDriverController.R2()
                                .onTrue(getJogTurnArmCommand());

        }

        private void configLeonardoBoxButtons() {

                m_codriverBox.L1().onTrue(new InstantCommand(() -> m_linArm.setTestMode()))
                                .onTrue(new InstantCommand(() -> m_turnArm.setTestMode()))
                                .onTrue(new InstantCommand(() -> m_grp.setTestMode()));

        }

        public Command getDriveCommand() {
                return new SetSwerveDrive(m_drive,
                                () -> m_driverController.getLeftY(),
                                () -> m_driverController.getLeftX(),
                                () -> m_driverController.getRightX());

        }

       

        public Command getStrafeToTargetCommand() {

                return new StrafeToSlot(m_drive, m_ghs, m_llv, () -> m_driverController.getRawAxis(0))
                                .andThen(() -> m_drive.stopModules());
        }

        public Command getJogTurnArmCommand() {

                return new JogTurnArm(m_turnArm, () -> -m_coDriverController.getLeftY());
        }

        public Command getJogLinearArmCommand() {

                return new JogLinearArm(m_linArm, () -> m_coDriverController.getLeftX());
        }

        public Command getStopDriveCommand() {
                return new InstantCommand(() -> m_drive.stopModules());
        }

        public Command setTargetGrid(int n) {
                return new InstantCommand(() -> m_ghs.setActiveDropNumber(n));
        }

        public void simulationPeriodic() {

                m_fieldSim.periodic();
        }

        public void periodic() {
                m_fieldSim.periodic();

        }
}
