// Copyright (c) 2024-2025 Az-FIRST
// http://github.com/AZ-First
// Copyright (c) 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Cameras.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AprilTagConstants.AprilTagLayoutType;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScoreSide;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.composition.AutoFeed;
import frc.robot.commands.composition.AutoScore;
import frc.robot.commands.composition.ClearAlgae;
import frc.robot.commands.composition.Feed;
import frc.robot.commands.composition.Score;
import frc.robot.commands.composition.ScoreNoAlign;
import frc.robot.commands.indexer.RunIndexerBackwordCommand;
import frc.robot.subsystems.accelerometer.Accelerometer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.squidward.Squidward;
import frc.robot.subsystems.squidward.SquidwardIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.GetJoystickValue;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.OverrideSwitches;
import frc.robot.util.PowerMonitoring;
import frc.robot.util.RBSIEnum;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** This is the location for defining robot hardware, commands, and controller button bindings. */
public class RobotContainer {
  final CommandXboxController driverController = new CommandXboxController(0); // Main Driver
  final CommandXboxController operatorController = new CommandXboxController(1); // Second Operator

  final OverrideSwitches overrides = new OverrideSwitches(2); // Console toggle switches

  /** Declare the robot subsystems here ************************************ */
  // These are the "Active Subsystems" that the robot controlls
  private final Drive m_drivebase;

  // These are "Virtual Subsystems" that report information but have no motors
  private final Accelerometer m_accel;
  private final Vision m_vision;
  private final PowerMonitoring m_power;
  private final Elevator m_Elevator;
  private Twist2d robotVelocity = new Twist2d();
  private final Indexer m_indexer;
  private final Squidward m_squid;

  /** Dashboard inputs ***************************************************** */
  // AutoChoosers for both supported path planning types
  private final LoggedDashboardChooser<Command> autoChooserPathPlanner;

  private final LoggedDashboardChooser<ElevatorPosition> autoLevelChooser;

  private final AutoChooser autoChooserChoreo;
  private final AutoFactory autoFactoryChoreo;
  // Input estimated battery capacity (if full, use printed value)
  private final LoggedTunableNumber batteryCapacity =
      new LoggedTunableNumber("Battery Amp-Hours", 18.0);

  // Alerts
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.INFO);

  @AutoLogOutput private ScoreSide scoreSide = ScoreSide.RIGHT;

  /**
   * Constructor for the Robot Container. This container holds subsystems, opertator interface
   * devices, and commands.
   */
  public RobotContainer() {

    // Instantiate Robot Subsystems based on RobotType
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // YAGSL drivebase, get config from deploy directory
        m_drivebase = new Drive(this);
        m_Elevator =
            new Elevator(
                new ElevatorIOTalonFX(),
                () ->
                    driverController.getLeftTriggerAxis() * 6
                        - driverController.getRightTriggerAxis() * 6);
        m_vision =
            switch (Constants.getVisionType()) {
              case PHOTON ->
                  new Vision(
                      m_drivebase::addVisionMeasurement,
                      new VisionIOPhotonVision(camera1Name, robotToCamera1),
                      new VisionIOPhotonVision(camera2Name, robotToCamera2)
                      // new VisionIOPhotonVision(camera3Name, robotToCamera3),
                      // new VisionIOPhotonVision(camera4Name, robotToCamera4)
                      );
              case LIMELIGHT ->
                  new Vision(
                      m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
              case NONE ->
                  new Vision(
                      m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
              default -> null;
            };
        m_accel = new Accelerometer(m_drivebase.getGyro());
        m_indexer = new Indexer(new IndexerIOTalonFX());
        m_squid = new Squidward(new SquidwardIOTalonFX(), () -> 0.0);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drivebase = new Drive(this);

        m_Elevator =
            new Elevator(
                new ElevatorIOTalonFX(),
                () ->
                    driverController.getLeftTriggerAxis() * 6
                        - driverController.getRightTriggerAxis() * 6);
        m_vision =
            new Vision(
                m_drivebase::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drivebase::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, m_drivebase::getPose)
                // new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, m_drivebase::getPose),
                // new VisionIOPhotonVisionSim(camera4Name, robotToCamera4, m_drivebase::getPose)
                );
        m_accel = new Accelerometer(m_drivebase.getGyro());
        m_indexer = new Indexer(new IndexerIOTalonFX());
        m_squid = new Squidward(new SquidwardIOTalonFX(), () -> 0.0);
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drivebase = new Drive(this);

        m_Elevator =
            new Elevator(
                new ElevatorIOTalonFX(),
                () ->
                    driverController.getLeftTriggerAxis() * 6
                        - driverController.getRightTriggerAxis() * 6);
        m_vision =
            new Vision(m_drivebase::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_accel = new Accelerometer(m_drivebase.getGyro());
        m_indexer = new Indexer(new IndexerIOTalonFX());
        m_squid = new Squidward(new SquidwardIOTalonFX(), () -> 0.0);
        break;
    }

    NamedCommands.registerCommand(
        "Test",
        Commands.runOnce(
            () -> {
              System.out.println("Running Command");
            },
            m_indexer));

    NamedCommands.registerCommand("Score2", new ScoreNoAlign(m_Elevator, m_indexer, this));
    NamedCommands.registerCommand("Score", new AutoScore(m_Elevator, m_indexer, m_drivebase, this));
    NamedCommands.registerCommand(
        "AutoFeed", new AutoFeed(this, m_drivebase, m_indexer, m_Elevator));

    // In addition to the initial battery capacity from the Dashbaord, ``PowerMonitoring`` takes all
    // the non-drivebase subsystems for which you wish to have power monitoring; DO NOT include
    // ``m_drivebase``, as that is automatically monitored.
    m_power = new PowerMonitoring(batteryCapacity, m_indexer);

    // Set up the SmartDashboard Auto Chooser based on auto type
    switch (Constants.getAutoType()) {
      case PATHPLANNER:
        autoChooserPathPlanner =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        autoLevelChooser = new LoggedDashboardChooser<>("Auto Level");
        autoLevelChooser.addDefaultOption("L2", ElevatorPosition.L2);
        autoLevelChooser.addOption("L4", ElevatorPosition.L4);
        // Set the others to null
        autoChooserChoreo = null;
        autoFactoryChoreo = null;
        break;

      case CHOREO:
        autoFactoryChoreo =
            new AutoFactory(
                m_drivebase::getPose, // A function that returns the current robot pose
                m_drivebase::resetOdometry, // A function that resets the current robot pose to the
                // provided Pose2d
                m_drivebase::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                m_drivebase // The drive subsystem
                );
        autoChooserChoreo = new AutoChooser();
        autoChooserChoreo.addRoutine("twoPieceAuto", this::twoPieceAuto);
        // Set the others to null
        autoChooserPathPlanner = null;
        autoLevelChooser = new LoggedDashboardChooser<>("Auto Level");
        autoLevelChooser.addDefaultOption("L2", ElevatorPosition.L2);
        autoLevelChooser.addOption("L4", ElevatorPosition.L4);
        break;

      default:
        // Then, throw the error
        throw new RuntimeException(
            "Incorrect AUTO type selected in Constants: " + Constants.getAutoType());
    }

    // Define Auto commands
    defineAutoCommands();
    // Define SysIs Routines
    definesysIdRoutines();
    // Configure the button and trigger bindings
    configureBindings();
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  /** Use this method to define your Autonomous commands for use with PathPlanner / Choreo */
  private void defineAutoCommands() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {

    // --- DRIVE BINDINGS --- //
    GetJoystickValue driveStickY;
    GetJoystickValue driveStickX;
    GetJoystickValue turnStickX;
    if (OperatorConstants.kDriveLeftTurnRight) {
      driveStickY = driverController::getLeftY;
      driveStickX = driverController::getLeftX;
      turnStickX = driverController::getRightX;
    } else {
      driveStickY = driverController::getRightY;
      driveStickX = driverController::getRightX;
      turnStickX = driverController::getLeftX;
    }

    // --- SET STANDARD DRIVING AS DEFAULT COMMAND FOR THE DRIVEBASE --- //
    m_drivebase.setDefaultCommand(
        DriveCommands.fieldRelativeDrive(
            m_drivebase,
            () -> -driveStickY.value(),
            () -> -driveStickX.value(),
            () -> -turnStickX.value()));

    // --- DRIVER CONTROLLER BINDINGS --- //

    driverController.a().whileTrue(new Feed(this, m_drivebase, m_indexer, m_Elevator));
    driverController
        .x()
        .onTrue(new Score(m_Elevator, m_indexer, m_drivebase, this, () -> scoreSide));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  terminateAll();
                },
                m_Elevator));
    driverController.y().onTrue(new ClearAlgae(m_drivebase, this, m_squid));

    driverController.povUp().onTrue(new ScoreNoAlign(m_Elevator, m_indexer, this));
    driverController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setHoming(true);
                  m_Elevator.goHome().schedule();
                }));
    driverController.povLeft().onTrue(Commands.runOnce(m_drivebase::stopWithX, m_drivebase));
    driverController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_drivebase.resetPose(
                      (new Pose2d(m_drivebase.getPose().getTranslation(), new Rotation2d())));
                }));

    driverController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setManualOverRide(!m_Elevator.getManualOverride());
                },
                m_Elevator));

    // --- OPERATOR CONTROLLER BINDINGS --- //

    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setSelectedPosition(ElevatorPosition.STOWED);
                },
                m_Elevator));
    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  scoreSide = ScoreSide.RIGHT;
                }));
    operatorController
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  scoreSide = ScoreSide.LEFT;
                }));

    operatorController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setSelectedPosition(ElevatorPosition.L4);
                },
                m_Elevator));
    operatorController
        .povDown()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setSelectedPosition(ElevatorPosition.L1);
                },
                m_Elevator));
    operatorController
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setSelectedPosition(ElevatorPosition.L2);
                },
                m_Elevator));
    operatorController
        .povRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  m_Elevator.setSelectedPosition(ElevatorPosition.L3);
                },
                m_Elevator));

    operatorController.leftTrigger().whileTrue(new RunIndexerBackwordCommand(m_indexer));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandPathPlanner() {
    // Use the ``autoChooser`` to define your auto path from the SmartDashboard
    return autoChooserPathPlanner.get();
  }

  public ElevatorPosition getAutonomousScoreLevel() {
    return autoLevelChooser.get();
  }

  public Elevator getElevator() {
    return m_Elevator;
  }

  public ScoreSide getScoreSide() {
    return scoreSide;
  }

  public void terminateAll() {
    CommandScheduler.getInstance().cancelAll();
    m_drivebase.clearAutoAlignGoal();
    m_Elevator.goHome().schedule();
    m_indexer.stopVoltage();
    m_indexer.setExtended(false);
    m_squid.runPosition(0.0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void getAutonomousCommandChoreo() {
    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooserChoreo);

    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooserChoreo.selectedCommandScheduler());
  }

  /** Set the motor neutral mode to BRAKE / COAST for T/F */
  public void setMotorBrake(boolean brake) {
    m_drivebase.setMotorBrake(brake);
  }

  @AutoLogOutput(key = "RobotState/FieldVelocity")
  public Twist2d fieldVelocity(Rotation2d rotation) {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(rotation);
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  /** Updates the alerts. */
  public void updateAlerts() {
    // AprilTag layout alert
    boolean aprilTagAlertActive = Constants.getAprilTagLayoutType() != AprilTagLayoutType.OFFICIAL;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-official AprilTag layout in use ("
              + Constants.getAprilTagLayoutType().toString()
              + ").");
    }
  }

  /**
   * Set up the SysID routines from AdvantageKit
   *
   * <p>NOTE: These are currently only accessible with Constants.AutoType.PATHPLANNER
   */
  private void definesysIdRoutines() {
    if (Constants.getAutoType() == RBSIEnum.AutoType.PATHPLANNER) {
      // Drivebase characterization
      autoChooserPathPlanner.addOption(
          "Drive Wheel Radius Characterization",
          DriveCommands.wheelRadiusCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive Simple FF Characterization",
          DriveCommands.feedforwardCharacterization(m_drivebase));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Forward)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Quasistatic Reverse)",
          m_drivebase.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Forward)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooserPathPlanner.addOption(
          "Drive SysId (Dynamic Reverse)",
          m_drivebase.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  /**
   * Example Choreo auto command
   *
   * <p>NOTE: This would normally be in a spearate file.
   */
  private AutoRoutine twoPieceAuto() {
    AutoRoutine routine = autoFactoryChoreo.newRoutine("twoPieceAuto");

    // Load the routine's trajectories
    AutoTrajectory pickupTraj = routine.trajectory("pickupGamepiece");
    AutoTrajectory scoreTraj = routine.trajectory("scoreGamepiece");

    // When the routine begins, reset odometry and start the first trajectory
    routine.active().onTrue(Commands.sequence(pickupTraj.resetOdometry(), pickupTraj.cmd()));

    // Starting at the event marker named "intake", run the intake
    // pickupTraj.atTime("intake").onTrue(intakeSubsystem.intake());

    // When the trajectory is done, start the next trajectory
    pickupTraj.done().onTrue(scoreTraj.cmd());

    // While the trajectory is active, prepare the scoring subsystem
    // scoreTraj.active().whileTrue(scoringSubsystem.getReady());

    // When the trajectory is done, score
    // scoreTraj.done().onTrue(scoringSubsystem.score());

    return routine;
  }
}
