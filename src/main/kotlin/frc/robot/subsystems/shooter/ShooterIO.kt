package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import org.team9432.annotation.Logged

interface ShooterIO {
    val inputs: LoggedShooterInputs

    fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {}
    fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {}
    fun updateInput() {}

    @Logged
    open class ShooterInputs {
        var topVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
        var bottomVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
        var topVoltage:Measure<Voltage> = Units.Volt.zero()
        var bottomVoltage:Measure<Voltage> = Units.Volt.zero()
    }
}