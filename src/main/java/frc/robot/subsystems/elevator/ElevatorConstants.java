package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.arm.ArmConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Units.*;

public class ElevatorConstants {
        public static final int kMotorPort = 0;
        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;
        public static final int kJoystickPort = 0;

        public static final MechanismRoot2d kElevatorCarriage;
        public static final MechanismLigament2d kElevatorTower;

        public static double kElevatorKp = 6;
        public static double kElevatorKi = 4;
        public static double kElevatorKd = 3;

        public static double kElevatorkS = 0; // volts (V)
        public static double kElevatorkV = 0; // volt per velocity (V/(m/s))
        public static double kElevatorkA = 0; // volt per acceleration (V/(m/s²))
        public static double kElevatorkG = 2; // volts (V)

        public static double kMaxVelocity = Meters.of(5).per(Second).in(MetersPerSecond);
        public static double kMaxAcceleration = Meters.of(5).per(Second).per(Second).in(MetersPerSecondPerSecond);

        //Motor rotation retio
        public static final double kElevatorGearing = 10.0;
        //f
        public static final double kElevatorDrumRadius = 0.2;
        public static final double kCarriageMass = 4.0; // kg

        // Encoder is reset to measure 0 at the bottom, so minimum height is 0.
        //public static final Distance kLaserCANOffset = Inches.of(3);
        public static final Distance kStartingHeightSim = Meters.of(0);
        public static final Distance kMinElevatorHeight = Meters.of(0.0);
        public static final Distance kMaxElevatorHeight = Meters.of(2);

        public static double kElevatorRampRate = 0.1;
        public static int kElevatorCurrentLimit = 40;

        public static final Mechanism2d sideRobotView = new Mechanism2d(ArmConstants.kArmLength * 2,
                        ElevatorConstants.kMaxElevatorHeight.in(Meters));
        static {
                kElevatorCarriage = ElevatorConstants.sideRobotView.getRoot("ElevatorCarriage", ArmConstants.kArmLength,
                                ElevatorConstants.kStartingHeightSim.in(Meters));
                kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
                                "Elevator",
                                ElevatorConstants.kStartingHeightSim.in(Meters),
                                -90,
                                6,
                                new Color8Bit(Color.kRed)));
        }

        //ppublic static final Mechanism2d setpoint = new Mechanism2d(0.3, 0.1);

        public static double[] elevatorStates = {
                0, //min
                0.2,
                0.5,
                1,
                1.9, //max
        };

}
