// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.SwerveDrive.kMaxSpeedMetersPerSecond;
import static frc.robot.Constants.SwerveModule.*;
import static frc.robot.Constants.SwerveModule.kDriveMotorGearRatio;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.unmanaged.Unmanaged;
import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.*;

public class SwerveModule extends SubsystemBase {
  int m_moduleNumber;
  CANSparkMax m_turnMotor;
  CANSparkMax m_driveMotor;
  CANCoder m_angleEncoder;
  double m_angleOffset;
  double m_lastAngle;
  Pose2d m_pose;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SwerveModule.ksDriveVoltSecondsPerMeter,
          Constants.SwerveModule.kvDriveVoltSecondsSquaredPerMeter,
          Constants.SwerveModule.kaDriveVoltSecondsSquaredPerMeter);

  private final FlywheelSim m_turnMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.1, 0.0008), kTurnGearbox, kTurningMotorGearRatio);

  private final FlywheelSim m_driveMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(4, 1.24), kDriveGearbox, kDriveMotorGearRatio);

  private double m_drivePercentOutput;
  private double m_turnPercentOutput;
  private double m_driveMotorSimDistance;
  private double m_turnMotorSimDistance;

  public SwerveModule(
      int moduleNumber,
      CANSparkMax turnMotor,
      CANSparkMax driveMotor,
      CANCoder angleEncoder,
      double angleOffset) {
    m_moduleNumber = moduleNumber;
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = angleOffset;

    m_driveMotor.restoreFactoryDefaults();
    RevUtils.setDriveMotorConfig(m_driveMotor);

    m_turnMotor.restoreFactoryDefaults();
    RevUtils.setTurnMotorConfig(m_turnMotor);

    m_angleEncoder.configFactoryDefault();
    m_angleEncoder.configAllSettings(CtreUtils.generateCanCoderConfig());

    REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));
//    REVPhysicsSim.getInstance().addSparkMax(m_turnMotor, DCMotor.getNEO(1));

    resetAngleToAbsolute();
  }

  public int getModuleNumber() {
    return m_moduleNumber;
  }

  public void resetAngleToAbsolute() {
    double angle = m_angleEncoder.getAbsolutePosition() - m_angleOffset;
    m_turnMotor.getEncoder().setPosition(angle / kTurnRotationsToDegrees);
  }

  public double getHeadingDegrees() {
    return m_turnMotor.getEncoder().getPosition() * kTurnRotationsToDegrees;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    return m_driveMotor.getEncoder().getVelocity() * kDriveRpmToMetersPerSecond;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;
      m_driveMotor.set(percentOutput);
    } else {
      double velocity = desiredState.speedMetersPerSecond / kDriveRpmToMetersPerSecond;
      m_driveMotor.getPIDController().setReference(
          velocity,
          CANSparkMax.ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond),
          SparkMaxPIDController.ArbFFUnits.kPercentOut
          );
    }

    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.getPIDController().setReference(angle / kTurnRotationsToDegrees,
            CANSparkMax.ControlType.kPosition, 0);
    m_lastAngle = angle;

//    System.out.printf("Turn %d Setpoint %.2f\n", m_moduleNumber, angle / kTurnRotationsToDegrees);
    System.out.printf("Turn %d Output %.2f\n", m_moduleNumber, m_turnMotor.get());
//    m_drivePercentOutput = m_driveMotor.get();
    m_turnPercentOutput = m_turnMotor.getPIDController().;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getHeadingRotation2d());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }

  private void updateSmartDashboard() {}

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {
    m_turnMotorSim.setInputVoltage(m_turnPercentOutput * RobotController.getBatteryVoltage());
//    m_driveMotorSim.setInputVoltage(m_drivePercentOutput * RobotController.getBatteryVoltage());

    m_turnMotorSim.update(0.02);
//    m_driveMotorSim.update(0.02);

    Unmanaged.feedEnable(20);

    m_turnMotorSimDistance = m_turnMotor.getEncoder().getPosition() + (m_turnMotorSim.getAngularVelocityRadPerSec() * 0.02 * 2 * Math.PI);
//    m_driveMotorSimDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02;
    m_turnMotor.getEncoder().setPosition(m_turnMotorSimDistance);
//    m_driveMotor
//        .getSimCollection()
//        .setIntegratedSensorRawPosition(
//            (int) (m_driveMotorSimDistance / kDriveMotorDistancePerPulse));
//    m_driveMotor
//        .getSimCollection()
//        .setIntegratedSensorVelocity(
//            (int)
//                (m_driveMotorSim.getAngularVelocityRadPerSec()
//                    / (kDriveMotorDistancePerPulse * 10)));
  }
}