// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
import frc.robot.commands.elevatorArm.algaeChaos;
import frc.robot.commands.elevatorArm.algeaState;
import frc.robot.commands.elevatorArm.runIntakeUntilCoral;
import frc.robot.commands.elevatorArm.safeElevator;
import frc.robot.commands.elevatorArm.setElevatorState;
import frc.robot.commands.elevatorArm.shootTillNoCoral;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ShooterSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.led.LedSubsystem;
//import frc.robot.subsystems.elevator.ElevatorSubsystem;
// garip hata import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
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


  private final SwerveSubsystem drivebase = new SwerveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve"));

  public final Elevator elevator = new Elevator();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  //private final ClimbSubsystem climb = new ClimbSubsystem();
  private final VisionSubsystem s_vision = new VisionSubsystem();
  private final LedSubsystem s_led = new LedSubsystem(elevator,shooter,s_vision);

  double swerveYawSpeed;
  double swerveSpeed;

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
      .scaleRotation(swerveYawSpeed)
      .scaleTranslation(swerveSpeed)
      .deadband(OperatorConstants.DEADBAND)
      .allianceRelativeControl(true);

      SwerveInputStream angularSlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driver1.getLeftY() * -1,
      () -> driver1.getLeftX() * -1)
      .withControllerRotationAxis(driver1::getRightX)
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(swerveSpeed)
      .scaleRotation(swerveYawSpeed/2)
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

    NamedCommands.registerCommand("go_L4", new setElevatorState(elevator, angleSubsystem, 4));
    NamedCommands.registerCommand("go_L3", new setElevatorState(elevator, angleSubsystem, 3));
    NamedCommands.registerCommand("go_L2", new setElevatorState(elevator, angleSubsystem, 2));
    NamedCommands.registerCommand("go_L1", new setElevatorState(elevator, angleSubsystem, 1));
    NamedCommands.registerCommand("shootTillNoCoral",shooter.ShooterShootSpecific(0.75).until(()->!shooter.IsCoral()));
    NamedCommands.registerCommand("intake", new runIntakeUntilCoral(shooter));
    NamedCommands.registerCommand("algaeChaos", new algaeChaos(elevator,angleSubsystem));


    // Autonomous Chooser (Searchs auto folder)
    autoChooser.setDefaultOption("do nothing", drivebase.getAutonomousCommand("do nothing"));
    autoChooser.addOption("hello", drivebase.getAutonomousCommand("hello"));
    autoChooser.addOption("firstl2", drivebase.getAutonomousCommand("firstl2"));
    autoChooser.addOption("taxi", drivebase.getAutonomousCommand("taxi"));

    autoChooser.addOption("l3king_last", drivebase.getAutonomousCommand("l3king_last"));
    autoChooser.addOption("l4king_middle", drivebase.getAutonomousCommand("l4king_middle"));
    autoChooser.addOption("l3king_1", drivebase.getAutonomousCommand("l3king_1"));
    autoChooser.addOption("l3king_2", drivebase.getAutonomousCommand("l3king_2"));
    autoChooser.addOption("l3king_3", drivebase.getAutonomousCommand("l3king_3"));

    autoChooser.addOption("AlgaeChaosUpper", drivebase.getAutonomousCommand("AlgaeChaosUpper"));
    autoChooser.addOption("AlgaeChaosBottom", drivebase.getAutonomousCommand("AlgaeChaosUpper"));
    autoChooser.addOption("short taxi", drivebase.getAutonomousCommand("short taxi"));
    autoChooser.addOption("go middle", drivebase.getAutonomousCommand("go middle"));
    autoChooser.addOption("middle l4", drivebase.getAutonomousCommand("middle4"));
    autoChooser.addOption("middle activity", drivebase.getAutonomousCommand("middle activity"));
  
  
    if (listOfAutos != null) {
      for (int i = 0; i < listOfAutos.length; i++) {
        if (listOfAutos[i].isFile()) {
          String newName = listOfAutos[i].getName().substring(0, listOfAutos[i].getName().length() - 5);
          autoChooser.addOption(newName, drivebase.getAutonomousCommand(newName));
        }
      }
    }

    SmartDashboard.putData(autoChooser);
  
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

    Command driveNstrike = drivebase.drive(driveAngularVelocity);



    Command driveManipulated = drivebase.driveCommand(
      ()->Preferences.getDouble("s_transX", 0),
      ()->Preferences.getDouble("s_transY", 0),
      ()->Preferences.getDouble("s_angular", 0)
      );

    


    elevator.setDefaultCommand(elevator.holdAtSetpoint());
    angleSubsystem.setDefaultCommand(angleSubsystem.HoldArmAtSetpoint());
    shooter.setDefaultCommand(shooter.OtReisStop());
    s_led.setDefaultCommand(s_led.LedCommand());

    angleSubsystem.SetAngle(ArmConstants.homeSetpoint);

    //climb.setDefaultCommand(climb.ClimbIdle());

    if (Robot.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    }
    if (DriverStation.isTest()) {

      drivebase.setDefaultCommand(driveNstrike); // Overrides drive command above!

    } else {

      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      driver2.povDown().whileTrue(elevator.manualDownCommand());
      driver2.povUp().whileTrue(elevator.manualUpCommand());
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

      driver2.leftTrigger().whileTrue(new runIntakeUntilCoral(shooter));
      driver2.rightTrigger().whileTrue(shooter.ShooterShootSpecific(0.75));

      driver2.a().onTrue(new setElevatorState(elevator,angleSubsystem,1));
      driver2.b().onTrue(new setElevatorState(elevator,angleSubsystem,2));
      driver2.y().onTrue(new setElevatorState(elevator,angleSubsystem,3));
      driver2.x().onTrue(new setElevatorState(elevator,angleSubsystem,4));

      driver2.leftBumper().onTrue(new algeaState(elevator, angleSubsystem, 4));//algea max
      driver2.rightBumper().onTrue(new algeaState(elevator, angleSubsystem, 3));//algea min
      driver2.povLeft().onTrue(new algeaState(elevator, angleSubsystem, 2));//shoot
      driver2.povRight().onTrue(new algeaState(elevator, angleSubsystem, 1));//home

      driver2.back().onTrue(angleSubsystem.setArmSetpoint(0));
      driver2.start().onTrue(elevator.setSetpointOnce(0));

      //Driver 1 (Controller, swerve)p

      driver1.circle().onTrue(Commands.runOnce(()->drivebase.zeroGyro()));

      driver1.povDown().whileTrue(shooter.ShooterShootSpecific(0.1));
      driver1.povUp().whileTrue(shooter.ShooterShootSpecific(-0.1));

      driver1.povRight().whileTrue(s_vision.driveRightAllign(drivebase, driver1));
      driver1.povLeft().whileTrue(s_vision.driveLeftAllign(drivebase, driver1));

      driver1.triangle().whileTrue(shooter.ShooterShootSpecific(-0.5));
      driver1.povUp().onTrue(new safeElevator(elevator, angleSubsystem, 3.26, 1.14));

      driver1.R2().whileTrue(shooter.ShooterShootSpecific(0.75));
      driver1.L2().whileTrue(new runIntakeUntilCoral(shooter));

      driver1.R1()
      .onTrue(new setElevatorState(elevator,angleSubsystem,4));

      driver1.L1()
      .onTrue(new setElevatorState(elevator,angleSubsystem,1));

      driver1.cross().whileTrue(
        s_vision.logBothAllign()
      );

  

      //driver1.povLeft().whileTrue(climb.ClimbUp());
      //driver1.povRight().whileTrue(climb.ClimbDown());

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


}