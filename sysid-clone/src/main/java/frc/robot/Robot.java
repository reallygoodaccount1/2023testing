// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final WPI_TalonSRX talonFL = new WPI_TalonSRX(DriveConstants.TALON_FL_ID);
  private final CANSparkMax sparkML = new CANSparkMax(DriveConstants.SPARK_ML_ID, MotorType.kBrushed);
  private final RelativeEncoder leftEncoder = sparkML.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
  private final WPI_TalonSRX talonBL = new WPI_TalonSRX(DriveConstants.TALON_BL_ID);
  private final MotorControllerGroup leftGroup = new MotorControllerGroup(talonFL, sparkML, talonBL);

  private final CANSparkMax sparkFR = new CANSparkMax(DriveConstants.SPARK_FR_ID, MotorType.kBrushed);
  private final RelativeEncoder rightEncoder = sparkFR.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
  private final WPI_TalonSRX talonMR = new WPI_TalonSRX(DriveConstants.TALON_MR_ID);
  private final WPI_TalonSRX talonBR = new WPI_TalonSRX(DriveConstants.TALON_BR_ID);
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(sparkFR, talonMR, talonBR);

  private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  String mech;
  int ackNum = 0;
  String testType;
  double voltageCommand;
  double startTime;
  double gearing;
  boolean rotate;
  ArrayList<Double> data;

  public Robot() {
    super();
    sparkML.setInverted(true);
    sparkFR.setInverted(true);
    rightGroup.setInverted(true);

    talonFL.setNeutralMode(NeutralMode.Brake);
    sparkML.setIdleMode(IdleMode.kBrake);
    talonBL.setNeutralMode(NeutralMode.Brake);
    sparkFR.setIdleMode(IdleMode.kBrake);
    talonMR.setNeutralMode(NeutralMode.Brake);
    talonBR.setNeutralMode(NeutralMode.Brake);

    leftEncoder.setInverted(DriveConstants.LEFT_ENCODER_INVERTED);
    // measurement period defaults to 100ms
    // leave average depth at default
    rightEncoder.setInverted(DriveConstants.RIGHT_ENCODER_INVERTED);

    drive.setMaxOutput(DriveConstants.DRIVE_SPEED);

    // 20s * 200 samples/s * 9 doubles/sample
    data = new ArrayList<>(36000);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    int kHALThreadPriority = 40;
    int kThreadPriority = 15;
    if (RobotBase.isReal()) {
      Notifier.setHALThreadPriority(true, kHALThreadPriority);
      Threads.setCurrentThreadPriority(true, kThreadPriority);
    }
    double cpr = 8192;
    gearing = 1.0;
    double combinedCPR = cpr * gearing;
    SmartDashboard.putNumber("SysIdCPR", cpr);
    SmartDashboard.putNumber("SysIdGearing", gearing);
    SmartDashboard.putNumber("SysIdConversionFactor", combinedCPR);

    LiveWindow.disableAllTelemetry();
    SmartDashboard.putNumber("SysIdVoltageCommand", 0.0);
    SmartDashboard.putString("SysIdTestType", "");
    SmartDashboard.putString("SysIdTest", "");
    SmartDashboard.putBoolean("SysIdRotate", false);
    SmartDashboard.putBoolean("SysIdOverflow", false);
    SmartDashboard.putBoolean("SysIdWrongMech", false);
    SmartDashboard.putNumber("SysIdAckNumber", ackNum);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  public boolean isWrongMechanism() {
    return !mech.equals("Drivetrain");
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    mech = SmartDashboard.getString("SysIdTest", "");
    if (isWrongMechanism()) {
      SmartDashboard.putBoolean("SysIdWrongMech", true);
    }
    testType = SmartDashboard.getString("SysIdTestType", "");
    rotate = SmartDashboard.getBoolean("SysIdRotate", false);
    voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
    startTime = Timer.getFPGATimestamp();
    SmartDashboard.putString("SysIdTelemetry", "");
    ackNum = (int) SmartDashboard.getNumber("SysIdAckNumber", 0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double timestamp = Timer.getFPGATimestamp();
    double voltage = 0;
    if (!isWrongMechanism()) {
      if (testType.equals("Quasistatic")) {
        voltage = voltageCommand * (timestamp - startTime);
      } else if (testType.equals("Dynamic")) {
        voltage = voltageCommand;
      }
    }

    double leftVoltage = 0;
    leftVoltage += talonFL.getMotorOutputVoltage();
    leftVoltage += sparkML.getBusVoltage() * sparkML.getAppliedOutput();
    leftVoltage += talonBL.getMotorOutputVoltage();
    leftVoltage /= 3;

    double rightVoltage = 0;
    rightVoltage += sparkFR.getBusVoltage() * sparkFR.getAppliedOutput();
    rightVoltage += talonMR.getMotorOutputVoltage();
    rightVoltage += talonBR.getMotorOutputVoltage();
    rightVoltage /= 3;

    double leftPosition = leftEncoder.getPosition() / gearing;
    double rightPosition = rightEncoder.getPosition() / gearing;
    double leftVelocity = leftEncoder.getVelocity() / gearing / 60;
    double rightVelocity = rightEncoder.getVelocity() / gearing / 60;
    double measuredAngle = Units.degreesToRadians(ahrs.getAngle());
    double angularRate = Units.degreesToRadians(ahrs.getRate());
    data.addAll(
        Arrays.asList(timestamp, leftVoltage, rightVoltage, leftPosition, rightPosition, leftVelocity, rightVelocity,
            measuredAngle, angularRate));

    double lVoltage = (rotate ? -1 : 1) * voltage;
    double rVoltage = voltage;

    talonFL.setVoltage(lVoltage);
    sparkML.setVoltage(lVoltage);
    talonBL.setVoltage(lVoltage);

    sparkFR.setVoltage(rVoltage);
    talonMR.setVoltage(rVoltage);
    talonBR.setVoltage(rVoltage);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("Disabled, sending " + data.size() + " data points");
    String telemetry = data.stream().map(Object::toString).collect(Collectors.joining(","));
    String type = testType.equals("Dynamic") ? "fast" : "slow";
    String direction = voltageCommand > 0 ? "forward" : "backward";
    String test = type + "-" + direction;
    SmartDashboard.putString("SysIdTelemetry", test + ";" + telemetry);
    SmartDashboard.putNumber("SysIdAckNumber", ++ackNum);
    reset();
  }

  void reset() {
    startTime = 0;
    data.clear();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
