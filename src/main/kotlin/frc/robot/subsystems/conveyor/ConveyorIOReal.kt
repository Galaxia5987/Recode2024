package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
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
        roller.configurator.apply(ConveyorConstants.MOTOR_CONFIG)
    }

    override fun setVelocity(velocity: Measure<Velocity<Angle>>) {
        if (velocity == Units.RotationsPerSecond.of(0.0)){
            roller.stopMotor()
        }else {
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