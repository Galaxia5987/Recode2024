package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import frc.robot.Ports
import frc.robot.lib.Utils

class HoodIOReal : HoodIO {
    override val inputs = LoggedHoodInputs()
    private val motor: TalonFX = TalonFX(Ports.Hood.MOTOR_ID)
    private val encoder = TalonSRX(Ports.Hood.ENCODER_ID)

    private val angleControl = PositionTorqueCurrentFOC(0.0)

    init {
        val config = TalonFXConfiguration()
            .apply {
                Feedback = FeedbackConfigs().apply {
                    SensorToMechanismRatio = HoodConstants.GEAR_RATIO
                }
                MotionMagic = MotionMagicConfigs().apply {
                    MotionMagicCruiseVelocity = HoodConstants.MAX_VELOCITY.`in`(Units.RotationsPerSecond)
                }
                Slot0 = Slot0Configs().apply {
                    kP = HoodConstants.GAINS.kP
                    kI = HoodConstants.GAINS.kI
                    kD = HoodConstants.GAINS.kD
                    kS = HoodConstants.GAINS.kS
                    kV = HoodConstants.GAINS.kV
                    kA = HoodConstants.GAINS.kA
                    kG = HoodConstants.GAINS.kG
                    GravityType = GravityTypeValue.Arm_Cosine
                    StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign
                }
                MotorOutput = MotorOutputConfigs().apply {
                    Inverted = HoodConstants.INVERTED_VALUE
                }
                CurrentLimits = CurrentLimitsConfigs().apply {
                    StatorCurrentLimitEnable = true
                    SupplyCurrentLimitEnable = true
                    StatorCurrentLimit = 2 * HoodConstants.CURRENT_LIMIT
                    SupplyCurrentLimit = HoodConstants.CURRENT_LIMIT
                }
            }

        motor.configurator.apply(config)

        encoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
    }

    private fun getEncoderPosition(): Double {
        val encoderTicksPerRevolution = HoodConstants.ENCODER_TICKS_PER_REVOLUTION
        val encoderPosition = encoder.selectedSensorPosition % encoderTicksPerRevolution
        val normalizedPosition =
            encoderPosition / encoderTicksPerRevolution - HoodConstants.ABSOLUTE_ENCODER_OFFSET.get()
        return Utils.normalize(Rotation2d.fromRotations(normalizedPosition)).rotations
    }

    override fun updateInternalEncoder() {
        motor.setPosition(getEncoderPosition())
    }

    override fun setAngle(angle: Measure<Angle>) {
        val error = angle.minus(inputs.absoluteEncoderAngle)
        motor.setControl(
            angleControl
                .withPosition(inputs.internalAngle.plus(error).`in`(Units.Rotations))
        )
    }

    override fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double, kG: Double) {
        motor.configurator.apply(
            Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
                .withKG(kG)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        )
    }

    override fun updateInputs() {
        inputs.internalAngle.mut_replace(motor.position.value, Units.Rotations)
        inputs.absoluteEncoderAngle.mut_replace(getEncoderPosition(), Units.Rotations)
        inputs.voltage.mut_replace(motor.motorVoltage.value, Units.Volts)
        inputs.absoluteEncoderAngleNoOffset.mut_replace(
            ((encoder.getSelectedSensorPosition() % HoodConstants.ENCODER_TICKS_PER_REVOLUTION)
                    / HoodConstants.ENCODER_TICKS_PER_REVOLUTION),
            Units.Rotations
        )
    }
}