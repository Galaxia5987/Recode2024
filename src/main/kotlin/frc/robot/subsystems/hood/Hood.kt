package frc.robot.subsystems.hood

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Hood private constructor(private val io: HoodIO) : SubsystemBase() {
    private val inputs: LoggedHoodInputs = io.inputs
    @AutoLogOutput
    private var angleSetpoint: Measure<Angle> = Units.Rotations.zero()
    private val timer = Timer()
    private val encoderTimer = Timer()

    @AutoLogOutput
    private val mechanism2d = Mechanism2d(HoodConstants.SIMULATION_LENGTH, HoodConstants.SIMULATION_LENGTH)
    private val root = mechanism2d.getRoot("Hood", HoodConstants.MECHANISM_2D_POSE.x, HoodConstants.MECHANISM_2D_POSE.y)
    private val hood = root.append(
        MechanismLigament2d(
            "Hood", HoodConstants.HOOD_LENGTH.`in`(Units.Meters), 45.0
        )
    )

    companion object { // Custom Singleton Implementation
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

    init {
        timer.start()
        timer.reset()

        encoderTimer.start()
        encoderTimer.reset()
    }

    @AutoLogOutput
    fun atSetpoint(): Boolean =
        inputs.absoluteEncoderAngle.isNear(angleSetpoint, HoodConstants.MAX_TOLERANCE)

    fun getAngle(): MutableMeasure<Angle> = inputs.internalAngle

    fun setAngle(angle: Measure<Angle>): Command {
        return run {
            angleSetpoint = angle
            io.setAngle(angle)
        }.withName("Set Angle Hood")
    }

    fun setRestingAngle(): Command = setAngle(HoodConstants.RESTING_ANGLE.mutableCopy()).withName("Set Resting Angle Hood")

    @AutoLogOutput(key = "Hood/Pose")
    private fun getPose3d() : Pose3d = Pose3d(HoodConstants.ROOT_POSITION, Rotation3d(0.0, getAngle().plus(HoodConstants.SIMULATION_OFFSET).`in`(Units.Radians), 0.0))

    override fun periodic() {
        io.updateInputs()
        if (encoderTimer.advanceIfElapsed(0.5)) io.updateInternalEncoder()
        Logger.processInputs("Hood", inputs)

        hood.setAngle(inputs.internalAngle.`in`(Units.Degrees))
    }
}