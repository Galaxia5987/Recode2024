package frc.robot.subsystems.conveyor

import edu.wpi.first.units.*
import org.team9432.annotation.Logged

interface ConveyorIO {
    val inputs: LoggedConveyorInputs

    fun setVelocity(velocity: Measure<Velocity<Angle>>) {}

    fun updateInputs() {}

    fun stop() {}

    fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {}

    @Logged
    open class ConveyorInputs {
        var velocity: MutableMeasure<Velocity<Angle>> = MutableMeasure.zero(Units.RotationsPerSecond)
    }
}