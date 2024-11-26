package frc.robot.subsystems.hood

import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Hood private constructor(private var io: HoodIO) : SubsystemBase() {
    private val inputs = io.inputs

    @AutoLogOutput
    private var angleSetpoint: Measure<AngleUnit> = Units.Rotations.zero()

    companion object {
        @Volatile
        private var instance: Hood? = null

        fun initialize(io: HoodIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Hood(io)
                }
            }
        }

        fun getInstance(): Hood {
            return instance ?: throw IllegalStateException(
                "Hood has not been initialized. Call initialize(io: HoodIO) first."
            )
        }
    }

    fun setAngle(angle: Angle): Command = Commands.runOnce({
        io.setAngle(angle)
        angleSetpoint = angle
    }).withName("set Angle Hood")

    fun setRestAngle(): Command = Commands.runOnce({ io.setAngle(HoodConstants.REST_ANGLE) }).withName("setRestAngle")

    @AutoLogOutput
    fun atSetPoint(): Boolean = inputs.angle.isNear(angleSetpoint, HoodConstants.TOLERANCE)

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}