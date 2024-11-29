package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ShooterIO {
    val inputs: LoggedShooterInputs

    fun setTopVelocity(velocity: AngularVelocity) {}
    fun setBottomVelocity(velocity: AngularVelocity) {}
    fun updateInput() {}

    @Logged
    open class ShooterInputs {
        var topVelocity: AngularVelocity = Units.RotationsPerSecond.zero()
        var bottomVelocity: AngularVelocity = Units.RotationsPerSecond.zero()
        var topVoltage: Voltage = Units.Volt.zero()
        var bottomVoltage: Voltage = Units.Volt.zero()
    }
}