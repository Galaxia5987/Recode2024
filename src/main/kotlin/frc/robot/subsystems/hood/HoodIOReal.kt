package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import frc.robot.lib.Utils
import kotlin.math.sign

class HoodIOReal : HoodIO {
    override val inputs = LoggedHoodInputs()
    private val motor: TalonFX = TalonFX(-1) // TODO: Replace with real motor port
    private val encoder = TalonSRX(-1) // TODO: Replace with real Port

    private val positionControl = PositionTorqueCurrentFOC(0.0)

    init {
        motor.configurator.apply(HoodConstants.MOTOR_CONFIGURATION)

        encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
    }

    private fun getEncoderPosition(): Double {
        val encoderTicksPerRevolution = HoodConstants.ENCODER_TICKS_PER_REVOLUTION.toDouble()
        val encoderPosition = encoder.selectedSensorPosition % encoderTicksPerRevolution
        val normalizedPosition = encoderPosition / encoderTicksPerRevolution - HoodConstants.ABSOLUTE_ENCODER_OFFSET.get()
        return Utils.normalize(Rotation2d.fromRotations(normalizedPosition)).rotations
    }

    override fun updateInternalEncoder() {
        motor.setPosition(getEncoderPosition())
    }

    override fun setAngle(angle: MutableMeasure<Angle>) {
        val error = angle.minus(inputs.absoluteEncoderAngle)
        motor.setControl(
            positionControl
                .withPosition(inputs.internalAngle.plus(error).`in`(Units.Rotations))
                .withFeedForward(
                    sign(
                        inputs.angleSetpoint.minus(inputs.absoluteEncoderAngle
                        ).`in`(Units.Degrees)) * HoodConstants.kS.get())
        )
    }

    override fun updateInputs() {
        inputs.internalAngle.mut_replace(motor.position.value, Units.Rotations)
        inputs.absoluteEncoderAngle.mut_replace(getEncoderPosition(), Units.Rotations)
        inputs.voltage.mut_replace(motor.motorVoltage.value, Units.Volts)
        inputs.absoluteEncoderAngleNoOffset.mut_replace(
            ((encoder.getSelectedSensorPosition() % HoodConstants.ENCODER_TICKS_PER_REVOLUTION)
                    / HoodConstants.ENCODER_TICKS_PER_REVOLUTION),
            Units.Rotations)
    }
}