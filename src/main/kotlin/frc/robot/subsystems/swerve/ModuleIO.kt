package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.webconstants.LoggedTunableNumber
import org.team9432.annotation.Logged

interface ModuleIO {

    val inputs: LoggedModuleInputs

    var angle: Rotation2d
        get() = Rotation2d()
        set(angle) {}

    var velocity: Double
        get() = 0.0
        set(velocity) {}

    val moduleState: SwerveModuleState
        get() = SwerveModuleState()

    val modulePosition: SwerveModulePosition
        get() = SwerveModulePosition()

    fun updateInputs() {}

    fun updatePID() {}

    fun hasPIDChanged(PIDValues: Array<LoggedTunableNumber>): Boolean {
        var hasChanged = false
        for (value in PIDValues) {
            if (value.hasChanged()) hasChanged = true
        }
        return hasChanged
    }

    fun stop() {}

    fun checkModule(): Command? {
        return Commands.none()
    }

    fun updateOffset(offset: Rotation2d) {}

    fun setVoltage(volts: Double) {}

    @Logged
    open class ModuleInputs {
        var driveMotorVelocity: Double = 0.0
        var driveMotorVoltage: Double = 0.0
        var driveMotorVelocitySetpoint: Double = 0.0
        var driveMotorPosition: Double = 0.0
        var driveMotorAcceleration: Double = 0.0

        var angle: Rotation2d = Rotation2d()
        var angleSetpoint: Rotation2d = Rotation2d()
        var absolutePosition: Double = 0.0
        var angleMotorAppliedVoltage: Double = 0.0
        var angleMotorVelocity: Double = 0.0

        var moduleDistance: Double = 0.0
        var moduleState: SwerveModuleState = SwerveModuleState()

        var encoderHasFaults: Boolean = false
    }
}
