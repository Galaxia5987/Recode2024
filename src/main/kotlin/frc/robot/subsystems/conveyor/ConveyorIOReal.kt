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
        val config = TalonFXConfiguration().apply {
            MotorOutput = MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0 = Slot0Configs().apply {
                kP = ConveyorConstants.GAINS.kP
                kI = ConveyorConstants.GAINS.kI
                kD = ConveyorConstants.GAINS.kD
                kS = ConveyorConstants.GAINS.kS
                kV = ConveyorConstants.GAINS.kV
                kA = ConveyorConstants.GAINS.kA
            }
            CurrentLimits = CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
            Feedback = FeedbackConfigs().apply { SensorToMechanismRatio = ConveyorConstants.GEAR_RATIO
            }
        }

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

    override fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
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