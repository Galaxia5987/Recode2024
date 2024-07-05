package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure

class HoodIOReal : HoodIO {
    private val motor: TalonFX = TalonFX(-1) // TODO: Replace with real motor port
    private val encoder = (-1) // TODO: Replace with real Port
    private val positionControl = PositionTorqueCurrentFOC(0.0)

    init {
        motor.configurator.apply(HoodConstants.MOTOR_CONFIGURATION)

    }

    private fun getEncoderPosition() : Double = 0.0 // TODO: Implement

    override fun updateInternalEncoder() {
        // TODO: Implement
    }

    override fun setAngle(angle: MutableMeasure<Angle>) {
        // TODO: Implement
    }

    override fun setAngle(angle: MutableMeasure<Angle>, torqueChassisCompensation: Double) {
        // TODO: Implement
    }

    override fun updateInputs() {
        // TODO: Implement
    }
}