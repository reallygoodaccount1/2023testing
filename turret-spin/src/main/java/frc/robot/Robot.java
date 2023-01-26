// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>
 * Joystick analog values range from -1 to 1 and motor controller inputs also
 * range from -1 to 1
 * making it easy to work together.
 *
 * <p>
 * In addition, the encoder value of an encoder connected to ports 0 and 1 is
 * consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  SlewRateLimiter filter = new SlewRateLimiter(.5, -0.5, .2);
  private WPI_TalonSRX talonSRX = new WPI_TalonSRX(4);
  private WPI_TalonFX talonFX = new WPI_TalonFX(1);
  private CommandXboxController m_XboxController = new CommandXboxController(0);
  boolean toggle = false;

  @Override
  public void robotInit() {
    // configureButtonBindings();
    talonSRX.setNeutralMode(NeutralMode.Coast);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
    if (m_XboxController.getHID().getAButton()) {
      talonSRX.set(0.5);
    } 
    
    else if (m_XboxController.getHID().getBButton()) {
      talonSRX.set(-0.5);
    }
      else if (m_XboxController.getHID().getYButton()){
        if(toggle) {
          talonSRX.set(0.4);
          toggle = false;
        }
          else { 
            talonSRX.set(0.4);
            toggle = true;
          }
      }
   else {
      talonSRX.set(0);
    }
  }
}
