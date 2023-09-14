// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  VictorSP motorDerAdelante = new VictorSP(1);
  VictorSP motorDerAtras = new VictorSP(2);
  VictorSP motorIzqAdelante = new VictorSP(3);
  VictorSP motorIzqAtras = new VictorSP(4);
  VictorSP motorgarra1 = new VictorSP(6);
  VictorSP motorgarra2 = new VictorSP(7);


  MotorControllerGroup motoresDer = new MotorControllerGroup(motorDerAdelante, motorDerAtras);
  MotorControllerGroup motoresIzq = new MotorControllerGroup(motorDerAdelante, motorDerAtras);

  DifferentialDrive chasis = new DifferentialDrive(motoresIzq, motoresDer);

  double max_Speed = 0.75; 
  Joystick control1 = new Joystick(1);  
  

  Timer tiempo = new Timer();
  
  double tiempoDesdeInicio;
  double tiempoNormal;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  @Override
  public void robotInit() {

    tiempo.start();

  }

  @Override
  public void robotPeriodic() {
    tiempoDesdeInicio = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousInit() {
    double tiempoInicioAutonomo = Timer.getFPGATimestamp();

    tiempoNormal = tiempoDesdeInicio - tiempoInicioAutonomo;
  }

  @Override
  public void autonomousPeriodic() {
    if (tiempoNormal <4) {
      chasis.arcadeDrive(0.5, 0);
    }else if(tiempoNormal >= 4 && tiempoNormal <6){
      chasis.arcadeDrive(0, 0.2);

    }else if(tiempoNormal <= 6 && tiempoNormal <8 ){

      chasis.arcadeDrive(0.5, 0);
    }

    

    
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if(control1.getRawButton(1) == true){
      motorDerAdelante.set(max_Speed);

    }
    else motorDerAdelante.set(0);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
