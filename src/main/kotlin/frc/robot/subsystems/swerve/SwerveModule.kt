package frc.robot.subsystems.swerve

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class SwerveModule
    (private val io: ModuleIO, private val number: Int, private val offset: Double) : SubsystemBase() {

    private val timer = Timer()
    private val inputs = io.inputs

    init {
        timer.start()
        timer.reset()
    }

    fun setVelocity(velocity: Double) {
        var velocity = velocity
        val angleError = inputs.angleSetpoint.minus(inputs.angle)
        velocity *= angleError.cos
        io.velocity = velocity
    }

    var moduleState: SwerveModuleState
        get() = io.moduleState
        set(moduleState) {
            var moduleState = moduleState
            moduleState = SwerveModuleState.optimize(moduleState, inputs.angle)
            setVelocity(moduleState.speedMetersPerSecond)
            io.angle = moduleState.angle
        }

    val modulePosition
        get() = SwerveModulePosition(inputs.moduleDistance, inputs.angle)

    val position
        get() = inputs.absolutePosition

    val velocity
        get() = inputs.driveMotorVelocity

    fun stop() {
        io.stop()
    }

    fun checkModule(): Command? {
        return io.checkModule()
    }

    fun updateInputs() {
        io.updateInputs()
        if (timer.advanceIfElapsed(0.1)) {
            Logger.processInputs("module_$number", inputs)
        }
    }

    val acceleration
        get() = inputs.driveMotorAcceleration

    override fun periodic() {
        if (timer.advanceIfElapsed(2.0)) {
            io.updateOffset(Rotation2d(Units.rotationsToRadians(offset)))
        }
    }

    fun characterize(voltage: Double) {
        io.setVoltage(voltage)
        io.angle = Rotation2d()
    }

    fun updateSysIdRoutineLog(log: SysIdRoutineLog) {
        log.motor("$number")
            .voltage(edu.wpi.first.units.Units.Volts.of(inputs.angleMotorAppliedVoltage))
            .angularPosition(
                edu.wpi.first.units.Units.Rotations.of(inputs.angle.rotations)
            )
            .angularVelocity(
                edu.wpi.first.units.Units.RotationsPerSecond.of(
                    inputs.driveMotorVelocity
                            / (SwerveConstants.WHEEL_DIAMETER * Math.PI)
                )
            )
    }
}
