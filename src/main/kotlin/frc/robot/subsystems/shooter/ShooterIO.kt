package frc.robot.subsystems.shooter

import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.units.Voltage
import org.team9432.annotation.Logged

interface ShooterIO {
    val topRollerInputs: LoggedRollerInputs
        get() = LoggedRollerInputs()
    val bottomRollerInputs: LoggedRollerInputs
        get() = LoggedRollerInputs()

    fun setTopVelocity(velocity: MutableMeasure<Velocity<Angle>>) {}

    fun setBottomVelocity(velocity: MutableMeasure<Velocity<Angle>>) {}

    fun stop() {}

    fun updateInputs() {}

    @Logged
    open class RollerInputs {
        var velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
        var velocitySetpoint: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
        var voltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
    }
}