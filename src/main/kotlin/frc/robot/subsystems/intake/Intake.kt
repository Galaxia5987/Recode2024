package frc.robot.subsystems.intake

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Intake private constructor(private val io: IntakeIO) : SubsystemBase() {
    private val inputs = io.inputs

    companion object {
        @Volatile
        private var instance: Intake? = null

        fun initialize(io: IntakeIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Intake(io)
                }
            }
        }

        fun getInstance(): Intake {
            return instance ?: throw IllegalStateException(
                "Intake has not been initialized. Call initialize(io: IntakeIO) first."
            )
        }
    }

    fun setAngle(angle: Angle): Command = runOnce { io.setAngle(angle) }.withName("Set Angle Intake")

    fun resetAngle(): Command {
        return Commands.runOnce({ io.resetAngle() }).withName("resetAngle")
    }

    fun setAnglePower(power: Double): Command {
        return Commands.runOnce({ io.setAnglePower(power) }).withName("setAnglePower")
    }

    fun setSpinMotorPower(power: Double): Command {
        return Commands.runOnce({ io.setsSpinMotorPower(power) }).withName("setSpinMotorPower")
    }

    fun stopSpinMotor(): Command {
        return Commands.runOnce(io::stopSpinMotor).withName("StopSpin")
    }

    fun setCenterMotorPower(power: Double): Command {
        return Commands.runOnce({ io.setsCenterMotorPower(power) }).withName("setCenterMotorPower")
    }

    fun stopCenterMotor(): Command {
        return Commands.runOnce(io::stopCenterMotor).withName("StopCenterMotor")
    }

    fun stop(): Command = Commands.parallel(stopSpinMotor(), stopCenterMotor(), setAngle(IntakeConstants.UP_ANGLE))
    fun intakeIn(): Command = Commands.parallel(
        setSpinMotorPower(IntakeConstants.INTAKE_SPIN_POWER),
        setCenterMotorPower(IntakeConstants.INTAKE_CENTER_POWER)
    )

    fun intakeOut(): Command = Commands.parallel(
        setSpinMotorPower(-IntakeConstants.INTAKE_SPIN_POWER),
        setCenterMotorPower(-IntakeConstants.INTAKE_CENTER_POWER)
    )

    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}