package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Ports
import frc.robot.lib.LoggedTunableNumber

class ConveyorIOReal : ConveyorIO {
    override val inputs = LoggedConveyorInputs()
    private val roller = TalonFX(Ports.Conveyor.MOTOR_ID)
    private val control = VelocityVoltage(0.0).withEnableFOC(true)

    init {
        val config = TalonFXConfiguration()
            .withMotorOutput(
                MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive)
            ).withSlot0(
                Slot0Configs()
                    .withKP(ConveyorConstants.GAINS.kP)
                    .withKI(ConveyorConstants.GAINS.kI)
                    .withKD(ConveyorConstants.GAINS.kD)
                    .withKS(ConveyorConstants.GAINS.kS)
                    .withKV(ConveyorConstants.GAINS.kV)
                    .withKA(ConveyorConstants.GAINS.kA)
            ).withCurrentLimits(
                CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(80.0)
                    .withSupplyCurrentLimit(40.0)
            ).withFeedback(
                FeedbackConfigs()
                    .withSensorToMechanismRatio(ConveyorConstants.GEAR_RATIO)
            )

        roller.configurator.apply(config)
    }

    override fun setVelocity(velocity: Measure<Velocity<Angle>>) {
        if (velocity == Units.RotationsPerSecond.of(0.0)) {
            roller.stopMotor()
        } else {
            roller.setControl(control.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
        }
    }

    override fun stop() {
        roller.stopMotor()
    }

    override fun setPID(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
        roller.configurator.apply(
            Slot0Configs()
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKV(kV)
                .withKA(kA)
        )
    }

    override fun updateInputs() {
        inputs.velocity.mut_replace(
            roller.velocity.value, Units.RotationsPerSecond
        )
    }
}