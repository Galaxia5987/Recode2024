package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import org.team9432.annotation.Logged

interface ModuleIO {

    val inputs: LoggedModuleInputs

    var angle
        get() = Rotation2d()
        set(angle) {}

    var velocity
        get() = 0.0
        set(velocity) {}

    val moduleState
        get() = SwerveModuleState()

    val modulePosition
        get() = SwerveModulePosition()

    fun updateInputs() {}

    fun stop() {}

    fun checkModule(): Command? {
        return Commands.none()
    }

    fun updateOffset(offset: Rotation2d) {}

    fun setVoltage(volts: Double) {}

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

        var encoderHasFaults = false
    }
}
