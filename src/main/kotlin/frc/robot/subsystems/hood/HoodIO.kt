package frc.robot.subsystems.hood

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged
import java.util.function.Supplier

interface HoodIO {
    val inputs: LoggedHoodInputs

    fun updateInternalEncoder() {}

    fun setAngle(angle: Angle) {}

    fun setAngle(angle: Angle, torqueCompensation: Supplier<Double>) {}

    fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double, kG: Double) {}

    fun updateInputs() {}

    @Logged
    open class HoodInputs {
        var internalAngle: Angle = Units.Rotations.zero()
        var voltage: Voltage = Units.Volts.zero()
        var absoluteEncoderAngle: Angle = Units.Rotations.zero()
        var absoluteEncoderAngleNoOffset: Angle = Units.Rotations.zero()
    }
}