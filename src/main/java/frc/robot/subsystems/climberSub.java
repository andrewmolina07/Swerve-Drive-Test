// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class climberSub extends SubsystemBase {
  /** Creates a new climberSub. */
  CANSparkMax climberL,climberR;
  
  
  
  public climberSub() {
    climberL = new CANSparkMax(Constants.climberL,MotorType.kBrushless);
    climberR = new CANSparkMax(Constants.climberR,MotorType.kBrushless);

    climberL.restoreFactoryDefaults();
    climberR.restoreFactoryDefaults();
    climberL.setInverted(true); 
    climberR.setInverted(false);
    climberL.getEncoder().setPosition(0);
    climberR.getEncoder().setPosition(0);
  }

  public void setClimber(){
    if(climberL.getEncoder().getPosition() < 104) {
    climberL.set(1);
    } else {
      climberL.set(0);
    }
    if(climberR.getEncoder().getPosition() < 104) {
      climberR.set(1);
    } else {
      climberR.set(0);
    }
}

public void setClimberDown(){
  climberL.set(-1);
  climberR.set(-1);
}

public void stopClimber(){
  climberL.set(0);
  climberR.set(0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber L", climberL.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber R", climberR.getEncoder().getPosition());
  }
}
