package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import org.team9432.annotation.Logged

interface ShooterIO {
    val topRollerInputs: LoggedRollerInputs
    val bottomRollerInputs: LoggedRollerInputs

    fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {}

    fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {}

    fun stop() {}

    fun setTopGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {}

    fun setBottomGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {}

    fun updateInputs() {}

    @Logged
    open class RollerInputs {
        var velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
        var voltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
    }
}