package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Intake(private val io: IntakeIO) : SubsystemBase() {
    private val inputs = io.inputs

    fun setSpinPower(power: Double): Command {
        return Commands.run({
                io.setSpinPower(power)
            }
        )
    }

    fun setCenterPower(power: Double): Command {
        return Commands.run({
                io.setCenterPower(power)
            }
        )
    }

    fun setAngle(angle: Measure<Angle>): Command {
        return Commands.run({
                inputs.angleSetPoint = angle
                io.setAngle(angle)
            }
        )
    }

    fun intake(): Command {
        return Commands.parallel(
            setAngle(IntakeConstants.intakeAngle),
            setSpinPower(IntakeConstants.intakeSpinPower),
            setCenterPower(IntakeConstants.intakeCenterPower)
        )
    }

    fun stop(): Command {
        return Commands.parallel(
            setAngle(IntakeConstants.restAngle),
            setSpinPower(0.0),
            setCenterPower(0.0)
        )
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}