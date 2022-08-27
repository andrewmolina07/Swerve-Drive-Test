// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSub extends SubsystemBase {
  /** Creates a new intakeSub. */

CANSparkMax shooterB,intake;


  public intakeSub() {
// shooterB = new CANSparkMax(Constants.shooterB, MotorType.kBrushless);
intake = new CANSparkMax(Constants.intake, MotorType.kBrushless);
    
    // shooterB.restoreFactoryDefaults();
    // shooterB.setInverted(false);
  }

public void setIntake(){
//shooterB.set(0);
intake.set(.7);


}

public void setIntakeReverse(){
  // shooterB.set(-.7);
  intake.set(-.5);
  
  
  }

public void stopIntake(){
  // shooterB.set(0);
  intake.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
