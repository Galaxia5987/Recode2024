package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.Angle
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim

class ConveyorIOSim : ConveyorIO {
    override val inputs = LoggedConveyorInputs()
    private val control = VelocityVoltage(0.0)
    private val conveyor = TalonFXSim(
        1,
        ConveyorConstants.GEAR_RATIO,
        ConveyorConstants.MOMENT_OF_INERTIA.`in`(Units.Kilogram.mult(Units.Meters).mult(Units.Meters)),
        1.0
    )

    private val controller: PIDController =
        PIDController(ConveyorConstants.KP.get(), ConveyorConstants.KI.get(), ConveyorConstants.KD.get(), 0.02)
    private val feed: SimpleMotorFeedforward =
        SimpleMotorFeedforward(ConveyorConstants.KS.get(), ConveyorConstants.KV.get(), ConveyorConstants.KA.get())

    init {
        conveyor.setController(controller)
    }

    override fun setVelocity(velocity: MutableMeasure<Velocity<Angle>>) {
        conveyor.setControl(control.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        conveyor.setControl(control.withVelocity(0.0))
    }


    override fun updateInputs() {
        conveyor.update(Timer.getFPGATimestamp())
        inputs.velocity.mut_replace(conveyor.velocity, Units.RotationsPerSecond)
    }
}