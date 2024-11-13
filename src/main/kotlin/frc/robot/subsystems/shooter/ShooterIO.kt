package frc.robot.subsystems.shooter

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import org.team9432.annotation.Logged

interface ShooterIO {
    val inputs: LoggedShooterInputs

    fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {}
    fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {}
    fun updateInput()

    @Logged
    open class ShooterInputs {
        var topVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
        var buttomVelocity: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
    }
}