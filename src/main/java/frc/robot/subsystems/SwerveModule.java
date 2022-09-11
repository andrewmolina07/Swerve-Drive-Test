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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.*;

public class SwerveModule extends SubsystemBase {
  int m_moduleNumber;
  CANSparkMax m_turnMotor;
  CANSparkMax m_driveMotor;
  CANCoder m_angleEncoder;
  double m_angleOffset;
  double m_lastAngle;
  Pose2d m_pose;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController =
          new PIDController(1, 0, 0);

  SimpleMotorFeedforward feedforward =
          new SimpleMotorFeedforward(
                  Constants.SwerveModule.ksDriveVoltSecondsPerMeter,
                  Constants.SwerveModule.kvDriveVoltSecondsSquaredPerMeter,
                  Constants.SwerveModule.kaDriveVoltSecondsSquaredPerMeter);

  private final ProfiledPIDController m_turningPIDController
          = new ProfiledPIDController(1, 0, 0,
          new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));

  // Simulation stuff. These values are fake and will probably not reflect reality
  private final FlywheelSim m_turnMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(0.001, 0.1), kTurnGearbox, kTurningMotorGearRatio);

  private final FlywheelSim m_driveMotorSim =
      new FlywheelSim(
          // Sim Values
          LinearSystemId.identifyVelocitySystem(4, 1.24), kDriveGearbox, kDriveMotorGearRatio);

  private Encoder simulationTurnEncoder;
  private Encoder simulationThrottleEncoder;
  private EncoderSim simulationTurnEncoderSim;
  private EncoderSim simulationThrottleEncoderSim;

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

//    REVPhysicsSim.getInstance().addSparkMax(m_driveMotor, DCMotor.getNEO(1));
//    REVPhysicsSim.getInstance().addSparkMax(m_turnMotor, DCMotor.getNEO(1));
    if(RobotBase.isSimulation()) {
      switch (moduleNumber) {
        case 3:
          simulationTurnEncoder = new Encoder(15, 14);
          simulationThrottleEncoder = new Encoder(0, 1);
          break;
        case 2:
          simulationTurnEncoder = new Encoder(13, 12);
          simulationThrottleEncoder = new Encoder(2, 3);
          break;
        case 1:
          simulationTurnEncoder = new Encoder(11, 10);
          simulationThrottleEncoder = new Encoder(4, 5);
          break;
        case 0:
          simulationTurnEncoder = new Encoder(9, 8);
          simulationThrottleEncoder = new Encoder(6, 7);
          break;
      }
      simulationTurnEncoder.setDistancePerPulse(kTurningEncoderDistancePerPulse);
      simulationThrottleEncoder.setDistancePerPulse(kDriveRpmToMetersPerSecond);

      simulationTurnEncoderSim = new EncoderSim(simulationTurnEncoder);
      simulationThrottleEncoderSim = new EncoderSim(simulationThrottleEncoder);
    }

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
    if(RobotBase.isReal()) {
      return Rotation2d.fromDegrees(getHeadingDegrees());
    } else {
      return new Rotation2d(simulationTurnEncoder.getDistance());
    }
  }

  public double getVelocityMetersPerSecond() {
    if(RobotBase.isReal()) {
      return m_driveMotor.getEncoder().getVelocity() * kDriveRpmToMetersPerSecond;
    } else {
      return simulationThrottleEncoder.getRate();
    }
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = RevUtils.optimize(desiredState, getHeadingRotation2d());

    if(RobotBase.isReal()) {
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
    } else {
      m_drivePercentOutput = m_drivePIDController.calculate(
              getVelocityMetersPerSecond(), desiredState.speedMetersPerSecond);
      m_drivePercentOutput += feedforward.calculate(desiredState.speedMetersPerSecond);

      m_turnPercentOutput = m_turningPIDController.calculate(getHeadingRotation2d().getRadians(), desiredState.angle.getRadians());

      m_driveMotor.set(m_drivePercentOutput);
      m_turnMotor.set(m_turnPercentOutput);
    }
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
    // This code is all for WPILib Sim. Remove if RevLib updates their simulation code
    m_turnMotorSim.setInputVoltage(m_turnPercentOutput * RobotController.getBatteryVoltage());
    m_driveMotorSim.setInputVoltage(m_drivePercentOutput * RobotController.getBatteryVoltage());

    m_turnMotorSim.update(0.02);
    m_driveMotorSim.update(0.02);

    m_turnMotorSimDistance += m_turnMotorSim.getAngularVelocityRadPerSec() * 0.02;
    simulationTurnEncoderSim.setDistance(m_turnMotorSimDistance);
    simulationTurnEncoderSim.setRate(m_turnMotorSim.getAngularVelocityRadPerSec());

    m_driveMotorSimDistance += m_driveMotorSim.getAngularVelocityRadPerSec() * 0.02;
    simulationThrottleEncoderSim.setDistance(m_driveMotorSimDistance);
    simulationThrottleEncoderSim.setRate(m_driveMotorSim.getAngularVelocityRadPerSec());

  }
}