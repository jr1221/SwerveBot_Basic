// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final PWMSpeedController m_driveMotor;
  private final PWMSpeedController m_turningMotor;

  private final boolean PWMInverted;

  private final Encoder m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  /*
   * private final ProfiledPIDController m_turningPIDController = new
   * ProfiledPIDController( ModuleConstants.kModuleTurningP,
   * ModuleConstants.kModuleTurningI, ModuleConstants.kModuleTurningD, new
   * TrapezoidProfile.Constraints(
   * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
   * ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
   */
  private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kModuleTurningP,
      ModuleConstants.kModuleTurningI, ModuleConstants.kModuleTurningD);

  /**
   * Constructs a SwerveModule.
   *
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel,
      int[] turningEncoderPorts,
      boolean turningEncoderReversed, boolean isVictor, boolean shouldInvert) {
    if (isVictor) {
      m_driveMotor = new Victor(driveMotorChannel);
      m_turningMotor = new Victor(turningMotorChannel);
    } else {
      m_driveMotor = new Talon(driveMotorChannel);
      m_turningMotor = new Talon(turningMotorChannel);
    }

    this.PWMInverted = shouldInvert;

    this.m_turningEncoder = new Encoder(turningEncoderPorts[0], turningEncoderPorts[1]);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double returnWheelPowerWheelSpeed(double wheelPower) {

    double wheelRadius = Constants.ModuleConstants.kWheelDiameterMeters / 2;

    // v = radius * power
    double wheelSpeed = wheelRadius * wheelPower;
    return wheelSpeed;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));
    double wheelspeed = returnWheelPowerWheelSpeed(m_driveMotor.get());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(wheelspeed, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput = m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    int factor = PWMInverted ? -1 : 1;

  //   m_driveMotor.set(driveOutput*factor); // ORIGINAL

    m_driveMotor.setSpeed(state.speedMetersPerSecond * factor); // TODO: see driveOutput PID controller, which is drive
                                                                // motor
    m_turningMotor.set(turnOutput); // TODO: turn PID Controller
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_turningEncoder.reset();
  }
}