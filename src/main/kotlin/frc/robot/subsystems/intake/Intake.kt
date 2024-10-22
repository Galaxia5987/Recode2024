package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.LoggedTunableNumber
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Intake(private val io: IntakeIO) : SubsystemBase() {
    private val angleKP = LoggedTunableNumber("Intake/Angle/kP", IntakeConstants.GAINS.kP)
    private val angleKI = LoggedTunableNumber("Intake/Angle/kI", IntakeConstants.GAINS.kI)
    private val angleKD = LoggedTunableNumber("Intake/Angle/kD", IntakeConstants.GAINS.kD)

    @AutoLogOutput
    private var angleSetpoint: Measure<Angle> = Units.Degree.zero()
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
            return instance ?: throw IllegalArgumentException(
                "Intake has not been initialized. Call initialize(io: IntakeIO) first."
            )
        }
    }


    fun setSpinPower(power: Double): Command {
        return Commands.runOnce({
            io.setSpinPower(power)
        })
    }

    fun setCenterPower(power: Double): Command {
        return Commands.runOnce({
            io.setCenterPower(power)
        })
    }

    fun setAngle(angle: Measure<Angle>): Command {
        return Commands.runOnce({
            angleSetpoint = angle
            io.setAngle(angle)
        })
    }

    fun intake(): Command {
        return Commands.parallel(
            setAngle(IntakeConstants.INTAKE_ANGLE),
            setSpinPower(IntakeConstants.INTAKE_SPIN_POWER),
            setCenterPower(IntakeConstants.INTAKE_CENTER_POWER)
        )
    }

    fun outtake(): Command {
        return Commands.parallel(
            setAngle(IntakeConstants.REST_ANGLE),
            setSpinPower(-IntakeConstants.INTAKE_SPIN_POWER),
            setCenterPower(-IntakeConstants.INTAKE_CENTER_POWER)
        )
    }

    fun stopSpin(): Command {
        return Commands.parallel(
            setSpinPower(0.0), setCenterPower(0.0)
        )
    }

    fun stop(): Command {
        return setAngle(IntakeConstants.REST_ANGLE).alongWith(stopSpin())
    }

    fun reset(): Command {
        return Commands.run({ io.setAnglePower(-0.3) }).finallyDo(Runnable {
            io.resetEncoder()
            io.setAnglePower(0.0)
        })
    }

    override fun periodic() {
        LoggedTunableNumber.ifChanged(
            hashCode(), { kPID: DoubleArray ->
                io.setGains(
                    kPID[0], kPID[1], kPID[2]
                )
            }, angleKP, angleKI, angleKD
        )


        io.updateInputs()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}