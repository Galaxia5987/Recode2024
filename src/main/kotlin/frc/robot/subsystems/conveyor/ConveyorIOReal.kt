package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity

class ConveyorIOReal : ConveyorIO {
    private val roller = TalonFX(-1) // TODO: Replace with actual port
    private val control = VelocityVoltage(0.0).withEnableFOC(true)

    init {
        roller.configurator.apply(ConveyorConstants.MOTOR_CONFIG)
    }

    override fun setVelocity(velocity: MutableMeasure<Velocity<Angle>>) {
        roller.setControl(control.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        roller.stopMotor()
    }

    override fun updateInputs() {
        inputs.velocity.mut_replace(
            roller.velocity.value, Units.RotationsPerSecond
        )
    }
}