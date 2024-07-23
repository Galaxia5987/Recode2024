package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.team9432.annotation.Logged

@Logged
open class SwerveInputs {
    // x, y, omega
    var desiredSpeeds: ChassisSpeeds = ChassisSpeeds()

    var acceleration = 0.0

    var rawYaw = Rotation2d()
    var yaw = Rotation2d()
    var gyroOffset = Rotation2d()
}

@Logged
open class ModuleInputs {
    var driveMotorVelocity = 0.0
    var driveMotorVoltage = 0.0
    var driveMotorVelocitySetpoint = 0.0
    var driveMotorPosition = 0.0
    var driveMotorAcceleration = 0.0

    var angle = Rotation2d()
    var angleSetpoint = Rotation2d()
    var absolutePosition = 0.0
    var angleMotorAppliedVoltage = 0.0
    var angleMotorVelocity = 0.0

    var moduleDistance = 0.0
    var moduleState = SwerveModuleState()

    var encoderConnected = false
}