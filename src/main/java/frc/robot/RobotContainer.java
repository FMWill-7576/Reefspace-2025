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
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.arm.AngleSubsystem;
import frc.robot.subsystems.elevator.Elevator;
//import frc.robot.subsystems.elevator.ElevatorSubsystem;
// garip hata import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.BooleanSupplier;

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

  //private final SwerveSubsystem drivebase = new SwerveSubsystem(
    //  new File(Filesystem.getDeployDirectory(), "swerve/neo"));
 
  private final Elevator elevator = new Elevator();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();



  // Importing Auto's
  Path targetDir = Paths.get("").toAbsolutePath();;
  File autosFolder = targetDir.resolve("src/main/deploy/pathplanner/autos").toFile();
  File[] listOfAutos = autosFolder.listFiles();

  // Auto Chooser
  SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    // Autonomous Chooser (Searchs auto folder)
    /*
    autoChooser.setDefaultOption("do nothing", drivebase.getAutonomousCommand("do nothing"));
    SmartDashboard.putData(autoChooser);
    if (listOfAutos != null) {
      for (int i = 0; i < listOfAutos.length; i++) {
        if (listOfAutos[i].isFile()) {
          String newName = listOfAutos[i].getName().substring(0, listOfAutos[i].getName().length() - 5);
          autoChooser.addOption(newName, drivebase.getAutonomousCommand(newName));
        }
      }
    }
       */
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

    elevator.setDefaultCommand(elevator.elevHoldCommand());
    angleSubsystem.setDefaultCommand(angleSubsystem.armHoldAsAngle());

    if (Robot.isSimulation()) {
    
    }
    if (DriverStation.isTest()) {
    } else {

      
      driver2.x().onTrue(elevator.setgoal(0));
      //driver2.y().onTrue(elevator.setgoal(0.3));
      driver2.b().onTrue(elevator.setgoal(1));
      //driver2.b().onTrue(elevator.setgoal(4));
      
      driver2.povUp().whileTrue(elevator.manualUpCommand());
      driver2.povDown().whileTrue(elevator.manualDownCommand());

      driver2.a().onTrue(angleSubsystem.setAngleAsRotationCommand(Rotation2d.fromDegrees(0)));
      driver2.y().onTrue(angleSubsystem.setAngleAsRotationCommand(Rotation2d.fromDegrees(0.69)));

      //driver2.b().onTrue(angleSubsystem.resetEncoder());
  

      

      //driver2.leftBumper().onTrue(elevator.setStateDown());
      //driver2.leftBumper().onTrue(elevator.setStateUp());

      /*
      driver2.a().whileTrue(angleSubsystem.armUp());
      driver2.a().whileFalse(angleSubsystem.armStop());
      driver2.y().whileTrue(angleSubsystem.armDown());
      driver2.y().whileFalse(angleSubsystem.armStop());
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


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * // An example command will be run in autonomous
   * return Autos.exampleAuto(m_exampleSubsystem);
   * }
   */
  /*/
  public ParallelCommandGroup setElevArm(double goal) {
    return new ParallelCommandGroup(elevator.setElevatorPosition(goal));
  }
  */
  public void setDriveMode() {
    configureBindings();
  }
}