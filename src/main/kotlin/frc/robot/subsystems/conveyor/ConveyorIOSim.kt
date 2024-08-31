package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.units.*
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
        PIDController(ConveyorConstants.GAINS.kP, ConveyorConstants.GAINS.kI, ConveyorConstants.GAINS.kD, 0.02)
    private val feed: SimpleMotorFeedforward =
        SimpleMotorFeedforward(ConveyorConstants.GAINS.kS, ConveyorConstants.GAINS.kV, ConveyorConstants.GAINS.kA)

    init {
        conveyor.setController(controller)
    }

    override fun setVelocity(velocity: Measure<Velocity<Angle>>) {
        conveyor.setControl(control.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        conveyor.setControl(control.withVelocity(0.0))
    }

    override fun setGains(kP: Double, kI: Double, kD: Double, kS: Double, kV: Double, kA: Double) {
        conveyor.setController(PIDController(kP, kI, kD))
    }

    override fun updateInputs() {
        conveyor.update(Timer.getFPGATimestamp())
        inputs.velocity.mut_replace(conveyor.velocity, Units.RotationsPerSecond)
    }
}
