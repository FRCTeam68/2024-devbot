// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubSystem;
// import frc.robot.subsystems.LEDSubsystem;

public class RobotContainer {
  // private double MaxSpeed = 6; // 6 meters per second desired top speed
  // private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandXboxController m_xboxController = new CommandXboxController(0); // My joystick
  CommandPS4Controller m_ps4Controller = new CommandPS4Controller(1);
  // private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
  //     .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  //     .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  //                                                              // driving in open loop
  // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // private final Telemetry logger = new Telemetry(MaxSpeed);

  IntakeSubSystem m_Intake = new IntakeSubSystem();
  // LEDSubsystem m_LED = new LEDSubsystem();
  // DigitalInput m_noteSensor = new DigitalInput(0);
  // Trigger m_NoteSensorTrigger = new Trigger(m_noteSensor::get);

  public RobotContainer() {
    configureBindings();

       // Put subsystems to dashboard.
    // Shuffleboard.getTab("Drivetrain").add(m_robotDrive);
    Shuffleboard.getTab("IntakeSubsystem").add(m_Intake);

    // Log Shuffleboard events for command initialize, execute, finish, interrupt
    // shuffleboard has to be recording
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandExecute(
            command ->
                Shuffleboard.addEventMarker(
                    "Command executed", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
  }

  private void configureBindings() {
    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-m_xboxController.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-m_xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-m_xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

    // m_xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // m_xboxController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_xboxController.getLeftY(), -m_xboxController.getLeftX()))));

    // // reset the field-centric heading on left bumper press
    // m_xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    // drivetrain.registerTelemetry(logger::telemeterize);

    System.out.println("config bindings");

    // m_ps4Controller.L1().onTrue(m_Intake.setStateCommand(IntakeSubSystem.State.INTAKING_CONE));
    // m_ps4Controller.L2().onTrue(m_Intake.setStateCommand(IntakeSubSystem.State.INTAKING_CUBE));
    // m_ps4Controller.R2().onTrue(m_Intake.setStateCommand(IntakeSubSystem.State.PLACING));
    // m_ps4Controller.R1().onTrue(m_Intake.setStateCommand(IntakeSubSystem.State.IDLE));

    m_ps4Controller.circle().onTrue(Commands.runOnce(()->m_Intake.setState(IntakeSubSystem.State.TAKE_NOTE)));
    m_ps4Controller.square().onTrue(Commands.runOnce(()->m_Intake.setState(IntakeSubSystem.State.SPIT_NOTE)));
    m_ps4Controller.cross().onTrue(Commands.runOnce(()->m_Intake.setState(IntakeSubSystem.State.IDLE)));


    // m_NoteSensorTrigger.onTrue(Commands.run(()->m_LED.setShooter(true)))
    //                    .onFalse(Commands.run(()->m_LED.setShooter(false)));

  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
