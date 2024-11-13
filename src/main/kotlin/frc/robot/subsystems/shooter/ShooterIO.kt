package frc.robot.subsystems.shooter

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ShooterIO {
    val topRollerInputs: LoggedRollerInputs
    val bottomRollerInputs: LoggedRollerInputs

    fun setTopVelocity(velocity: AngularVelocity) {}

    fun setBottomVelocity(velocity: AngularVelocity) {}

    fun stop() {}

    fun setTopGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {}

    fun setBottomGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {}

    fun updateInputs() {}

    @Logged
    open class RollerInputs {
        var velocity: AngularVelocity = Units.RotationsPerSecond.zero()
        var voltage: Voltage = Units.Volts.zero()
    }
}
