// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.RawTopic;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.elevatorArm.safeElevator;
import frc.robot.commands.elevatorArm.setElevatorState;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.OtReisSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.led.LedSubsystem;
//import frc.robot.subsystems.elevator.ElevatorSubsystem;
// garip hata import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  final CommandPS5Controller driver1 = new CommandPS5Controller(0);
  final CommandXboxController driver2 = new CommandXboxController(1);

  public int elevState = 0;

  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));

  private final Elevator elevator = new Elevator();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final LedSubsystem s_led = new LedSubsystem(elevator);
  private final OtReisSubsystem otReis = new OtReisSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();

  // Importing Auto's
  Path targetDir = Paths.get("").toAbsolutePath();;
  File autosFolder = targetDir.resolve("src/main/deploy/pathplanner/autos").toFile();
  File[] listOfAutos = autosFolder.listFiles();

  // Auto Chooser
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  // SWERVELER

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver1.getLeftY() * -1,
      () -> driver1.getLeftX() * -1)
      .withControllerRotationAxis(driver1::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(Constants.OperatorConstants.swerveSpeed)
      .scaleRotation(Constants.OperatorConstants.swerveYawSpeed)
      .allianceRelativeControl(true);
      

    

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driver1::getRightX,
      driver1::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -driver1.getLeftY(),
      () -> -driver1.getLeftX())
      .withControllerRotationAxis(() -> driver1.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          driver1.getRawAxis(
              2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              driver1.getRawAxis(
                  2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Autonomous Chooser (Searchs auto folder)

    autoChooser.setDefaultOption("hello", drivebase.getAutonomousCommand("hello"));
    SmartDashboard.putData(autoChooser);
    if (listOfAutos != null) {
      for (int i = 0; i < listOfAutos.length; i++) {
        if (listOfAutos[i].isFile()) {
          String newName = listOfAutos[i].getName().substring(0, listOfAutos[i].getName().length() - 5);
          autoChooser.addOption(newName, drivebase.getAutonomousCommand(newName));
        }
      }
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);


    Command driveManipulated = drivebase.driveCommand(
      ()->Preferences.getDouble("s_transX", 0),
      ()->Preferences.getDouble("s_transY", 0),
      ()->Preferences.getDouble("s_angular", 0)
      );


    //elevator.setDefaultCommand(elevator.elevHoldCommand());
    //angleSubsystem.setDefaultCommand(angleSubsystem.armHoldAsAngle());
    otReis.setDefaultCommand(otReis.OtReisStop());

    angleSubsystem.SetAngle(ArmConstants.homeSetpoint);

    if (Robot.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    }
    if (DriverStation.isTest()) {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

    } else {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      driver2.povDown().whileTrue(elevator.manualDownCommand()).onFalse(elevator.manualStopCommand());
      driver2.povUp().whileTrue(elevator.manualUpCommand()).onFalse(elevator.manualStopCommand());
      //Set elevator as setpoints
    

      /*
      driver2.leftBumper()
        .onTrue(Commands.runOnce(()->{
          if(0<elevState){
            elevState-=1;
          }
        })
        .andThen(new safeElevator(elevator, angleSubsystem, ElevatorConstants.states[elevState])));

        driver2.rightBumper()
        .onTrue(Commands.runOnce(()->{
          if(elevState<ElevatorConstants.states.length){
            elevState+=1;
          }
        })
        .andThen(new safeElevator(elevator, angleSubsystem, ElevatorConstants.states[elevState])));
      */

      driver2.povLeft().onTrue(angleSubsystem.setAngleCommand(ArmConstants.homeSetpoint));
      driver2.povRight().onTrue(angleSubsystem.setAngleCommand(0.61));

      driver2.leftTrigger().whileTrue(otReis.OtReisIntake());
      driver2.rightTrigger().whileTrue(otReis.OtReisShooter());

      driver2.a().onTrue(new setElevatorState(elevator,angleSubsystem,1));
      driver2.b().onTrue(new setElevatorState(elevator,angleSubsystem,2));
      driver2.y().onTrue(new setElevatorState(elevator,angleSubsystem,3));
      driver2.x().onTrue(new setElevatorState(elevator,angleSubsystem,4));

      driver2.leftBumper().whileTrue(angleSubsystem.armDown()).whileFalse(angleSubsystem.armStop());
      driver2.rightBumper().whileTrue(angleSubsystem.armUp()).whileFalse(angleSubsystem.armStop());


      //Driver 1 (Controller, swerve)p

      //Yaw will follow the closest
      /*
      driver1.cross()
        .whileTrue(Commands.run(()->aimAtBestTargetPID()))
        .whileFalse(Commands.run(()->drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity)));

      driver1.circle().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      */


      driver1.cross()
        .whileTrue(vision.yawDrive(drivebase, driver1).onlyIf(()->vision.isAprilOnResult()));

      driver1.circle().onTrue(Commands.runOnce(()->drivebase.zeroGyro()));


      /*
      driver1.triangle()
        .whileTrue(new InstantCommand(()->this.aimAtBestTargetPID()))
        .onFalse(new InstantCommand(()->drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity)));
      */

      // driver2.b().onTrue(angleSubsystem.resetEncoder());

      // driver2.leftBumper().onTrue(elevator.setStateDown());
      // driver2.leftBumper().onTrue(elevator.setStateUp());

      /*
       * driver2.a().whileTrue(angleSubsystem.armUp());
       * driver2.a().whileFalse(angleSubsystem.armStop());
       * driver2.y().whileTrue(angleSubsystem.armDown());
       * driver2.y().whileFalse(angleSubsystem.armStop());
       */

    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public void aimAtBestTargetTest(){
    PhotonTrackedTarget id = vision.bestTarget();
    Translation2d translation = new Translation2d(id.getBestCameraToTarget().getMeasureX(),id.getBestCameraToTarget().getMeasureY());
    Rotation2d rotation = new Rotation2d(id.getYaw());
    Pose2d aimPose = new Pose2d(translation,rotation);
    Command driveAimCommand = drivebase.driveToPose(aimPose);

    drivebase.setDefaultCommand(driveAimCommand);
  }

  public void aimAtBestTargetPID() {
    PhotonTrackedTarget id = vision.bestTarget();

    double getGoal = MathUtil.clamp(vision.getLastestPID().calculate(drivebase.getPose().getRotation().getRadians(), Math.toRadians(id.getYaw())), -1, 1);

    SwerveInputStream driveVelocityAim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver1.getLeftY() * -1,
      () -> driver1.getLeftX() * -1)
      .withControllerRotationAxis(()->getGoal)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

    

    Command driveAimCommand = drivebase.driveFieldOriented(driveVelocityAim);
    drivebase.setDefaultCommand(driveAimCommand);
  }

  public void aimAtCurrentTagWithFieldYaw(){
    PhotonTrackedTarget id = vision.bestTarget();
    double radianYaw = vision.getAprilTagYawAsRadian(id.getFiducialId());
  }

}