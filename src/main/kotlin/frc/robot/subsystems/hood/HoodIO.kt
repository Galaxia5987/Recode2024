package frc.robot.subsystems.hood

import edu.wpi.first.units.*
import org.team9432.annotation.Logged
import java.util.function.Supplier

interface HoodIO {
    val inputs: LoggedHoodInputs

    fun updateInternalEncoder() {}

    fun setAngle(angle: Measure<Angle>) {}

    fun setAngle(angle: Measure<Angle>, torqueCompensation: Supplier<Double>) {}

    fun updateInputs() {}

    @Logged
    open class HoodInputs {
        var internalAngle: MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
        var voltage: MutableMeasure<Voltage> = MutableMeasure.zero(Units.Volts)
        var absoluteEncoderAngle: MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
        var absoluteEncoderAngleNoOffset: MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)
    }
}