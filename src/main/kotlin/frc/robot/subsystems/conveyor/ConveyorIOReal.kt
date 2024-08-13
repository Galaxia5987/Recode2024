package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import frc.robot.Ports

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

    override fun updateInputs() {
        inputs.velocity.mut_replace(
            roller.velocity.value, Units.RotationsPerSecond
        )

        if (hasPIDChanged(ConveyorConstants.PID_VALUES)) updatePID()
    }

    override fun updatePID() {
        val slot0Configs =
            Slot0Configs()
                .withKP(ConveyorConstants.KP.get())
                .withKI(ConveyorConstants.KI.get())
                .withKD(ConveyorConstants.KD.get())
                .withKS(ConveyorConstants.KS.get())
                .withKV(ConveyorConstants.KV.get())
                .withKA(ConveyorConstants.KA.get())

        roller.configurator.apply(slot0Configs)
    }
}