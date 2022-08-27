// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooterSub extends SubsystemBase {
  /** Creates a new shooterSub. */
  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-phantom");

  CANSparkMax shooterL,shooterR, shooterB;
  
  public shooterSub() {
    shooterL = new CANSparkMax(Constants.shooterL, MotorType.kBrushless);
    shooterR = new CANSparkMax(Constants.shooterR, MotorType.kBrushless);
    shooterB = new CANSparkMax(Constants.shooterB, MotorType.kBrushless);
    
    shooterL.restoreFactoryDefaults();
    shooterR.restoreFactoryDefaults();
    shooterB.restoreFactoryDefaults();
    
    shooterL.setInverted(false);
    shooterR.setInverted(false);
    shooterB.setInverted(false);
    
  }
  public void setShooterAuto(){
    
    shooterL.set(1);
    shooterR.set(-1);
    shooterB.set(.5);
  }
public void setShooter(){
  boolean hastarget = limelight.getEntry("tv").getDouble(0) == 1;
  if (hastarget) {
    double TargetDistance = getDistance();
    double Power = 0.00590681*TargetDistance+0.383986;
    shooterL.set(Power);
    shooterR.set(-Power);

  }
  else {
  shooterL.set(0.9);
  shooterR.set(-0.9);
  }
  shooterB.set(.5);
}

public double getDistance(){
  double limelightHeight = 27;
  double limelightMountingAngle = 45;
  double hubHeight = 104;
  double hubAngle = limelight.getEntry("ty").getDouble(0);
  boolean hastarget = limelight.getEntry("tv").getDouble(0) == 1;
  if (hastarget) {
    return (hubHeight - limelightHeight) /(Math.tan(Units.degreesToRadians(hubAngle+limelightMountingAngle)));
  
  }
  else {
  return 0;
  }
}

public void setShooterReverse(){
  shooterL.set(-.2);
  shooterR.set(.2);
  shooterB.set(-0.2);

}
public void stopShooter(){
  shooterL.set(0);
  shooterR.set(0);
  shooterB.set(0);

}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
