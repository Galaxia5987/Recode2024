package frc.robot.subsystems.telescopicArm

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.Ports

class TelescopicArmIOReal : TelescopicArmIO {
    override var inputs = LoggedTLArmInputs()
    val motor: TalonFX = TalonFX(Ports.TLArm.TL_MOTOR_ID)
    var controlRequest: PositionTorqueCurrentFOC = PositionTorqueCurrentFOC(0.0)
    override fun updateInputs() {
        inputs.currentPose =
            Units.Meters.of(motor.position.valueAsDouble)
        inputs.voltage = motor.supplyVoltage.value
    }

    val MOTOR_CONFIGURATION = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().apply {
            Inverted = InvertedValue.Clockwise_Positive
            NeutralMode = NeutralModeValue.Brake
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            val currentLimit = TelescopicArmConstants.CURRENT_LIMIT.`in`(Units.Amps)
            StatorCurrentLimit = currentLimit * 2
            SupplyCurrentLimit = currentLimit
            SupplyCurrentLimitEnable = true
            StatorCurrentLimitEnable = true
        }
        Slot0 = Slot0Configs().apply {
            kP = TelescopicArmConstants.KP
            kD = TelescopicArmConstants.KD
            kI = TelescopicArmConstants.KI
            kV = TelescopicArmConstants.KV
        }
        Feedback = FeedbackConfigs().apply {
            SensorToMechanismRatio =
                TelescopicArmConstants.CONVERSION_FACTOR * TelescopicArmConstants.DRUM_RADIUS.`in`(Units.Meters)
        }
    }

    init {
        motor.configurator.apply(MOTOR_CONFIGURATION)
    }

    override fun setHeight(distance: Distance) {
        motor.setControl(
            controlRequest.withPosition(
                distance.`in`(Units.Meters)

            )
        )
    }
}