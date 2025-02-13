// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
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
  final CommandPS5Controller driver2 = new CommandPS5Controller(1);

  //private final SwerveSubsystem drivebase = new SwerveSubsystem(
    //  new File(Filesystem.getDeployDirectory(), "swerve/neo"));
 
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

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

    // Elevator hold
    //elevator.setDefaultCommand(elevator.setAlternativeGoal());
    //elevator.setDefaultCommand(elevator.holdPosition());

    if (Robot.isSimulation()) {
      //driver1.back().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest()) {
    } else {
      // Put Mechanism 2d to SmartDashboard
      SmartDashboard.putData("Side View (Elevator)", ElevatorConstants.sideRobotView);

      // m_driverController.button(1).whileTrue(arm.setGoal(15));
      //driver1.button(1).whileTrue(elevator.setGoal(1));
      //driver1.y().whileTrue(elevator.setIncrementalGoal(true));
      //driver1.a().whileTrue(elevator.setIncrementalGoal(false));
      //driver1.button(2).whileTrue(elevator.setGoal(6));
      //driver1.button(2).whileTrue(arm.setGoal(45));
      //driver1.povUp().onTrue(elevator.setGoal(0.5).repeatedly().until(elevator.IsAtTheDesiredHeigh(1, 0.1)));
      //driver1.cross().onTrue(elevator.setGoal(0).repeatedly().until(elevator.IsAtTheDesiredHeigh(0, 0.1)));
      //driver1.triangle().onTrue(elevator.setGoal(1).repeatedly().until(elevator.IsAtTheDesiredHeigh(1, 0.1)));
      //driver1.circle().onTrue(elevator.setGoal(0.4).repeatedly().until(elevator.IsAtTheDesiredHeigh(0.5, 0.1)));
      //driver1.button(3).whileTrue(elevator.setGoal(9));
      //driver1.button(3).whileTrue(arm.setGoal(90));

      //driver1.button(4).whileTrue(arm.setGoal(135));

      // driver1.button(5).whileTrue(arm.runSysIdRoutine());
      //driver1.button(5).whileTrue(elevator.runSysIdRoutine());

      //driver1.button(6).whileTrue(arm.setGoal(70));
      //driver1.button(6).whileTrue(elevator.setGoal(4));
      // m_driverController.button(6).whileTrue(setElevArm(10, 70));

      //elevator.atHeight(1.9, 0.1).whileTrue(Commands.print("I AM ALIVE, YAAA HAAAAA"));
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
  public ParallelCommandGroup setElevArm(double goal, double degree) {
    return new ParallelCommandGroup(elevator.setGoal(goal));
  }

  public void setDriveMode() {
    configureBindings();
  }
}