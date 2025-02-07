package frc.robot.utils;

import frc.robot.Robot;

/**
 * Constants not specific to any given subsystem or commadn
 * 
 * @author Noah Simon
 */
public class MiscConstants {

  public static enum RobotType {
    REAL,
    SIM
  }

  public static final RobotType ROBOT_TYPE = Robot.isReal()
      ? RobotType.REAL
      : RobotType.SIM;

  public static final class SimConstants {
    public static final double loopPeriodSec = 0.02;
  }

  public static final class ControllerConstants {

    public static final double deadband = 0.05;
  }

}