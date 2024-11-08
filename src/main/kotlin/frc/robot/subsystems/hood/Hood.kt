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
import frc.robot.lib.LoggedTunableNumber
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Hood private constructor(private val io: HoodIO) : SubsystemBase() {
    private val kP = LoggedTunableNumber("Hood/kP", GAINS.kP)
    private val kI = LoggedTunableNumber("Hood/kI", GAINS.kI)
    private val kD = LoggedTunableNumber("Hood/kD", GAINS.kD)
    private val kS = LoggedTunableNumber("Hood/kS", GAINS.kS)
    private val kV = LoggedTunableNumber("Hood/kV", GAINS.kV)
    private val kA = LoggedTunableNumber("Hood/kA", GAINS.kA)
    private val kG = LoggedTunableNumber("Hood/kG", GAINS.kG)

    private val inputs: LoggedHoodInputs = io.inputs

    @AutoLogOutput
    private var angleSetpoint: Measure<Angle> = Units.Rotations.zero()
    private val timer = Timer()
    private val encoderTimer = Timer()

    @AutoLogOutput
    private val mechanism2d = Mechanism2d(SIMULATION_LENGTH, SIMULATION_LENGTH)
    private val root = mechanism2d.getRoot("Hood", MECHANISM_2D_POSE.x, MECHANISM_2D_POSE.y)
    private val hood = root.append(
        MechanismLigament2d(
            "Hood", HOOD_LENGTH.`in`(Units.Meters), 45.0
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
        inputs.absoluteEncoderAngle.isNear(angleSetpoint, MAX_TOLERANCE.`in`(Units.Percent))

    fun getAngle(): MutableMeasure<Angle> = inputs.internalAngle

    fun setAngle(angle: Measure<Angle>): Command {
        return runOnce {
            angleSetpoint = angle
            io.setAngle(angle)
        }.withName("Set Angle Hood")
    }

    fun setAngle(angleSupplier: () -> Measure<Angle>): Command {
        return run {
            val angle = angleSupplier.invoke()
            angleSetpoint = angle
            io.setAngle(angle)
        }.withName("Set Angle Hood")
    }

    fun setRestingAngle(): Command = setAngle(RESTING_ANGLE).withName("Set Resting Angle Hood")

    @AutoLogOutput(key = "Hood/Pose")
    private fun getPose3d(): Pose3d = Pose3d(
        ROOT_POSITION,
        Rotation3d(0.0, getAngle().plus(SIMULATION_OFFSET).`in`(Units.Radians), 0.0)
    )

    override fun periodic() {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            { kPIDSVAG: DoubleArray ->
                io.setGains(
                    kPIDSVAG[0],
                    kPIDSVAG[1],
                    kPIDSVAG[2],
                    kPIDSVAG[3],
                    kPIDSVAG[4],
                    kPIDSVAG[5],
                    kPIDSVAG[6]
                )
            },
            kP, kI, kD, kS, kV, kA, kG
        )

        io.updateInputs()
        if (encoderTimer.advanceIfElapsed(0.5)) io.updateInternalEncoder()
        Logger.processInputs("Hood", inputs)

        hood.setAngle(inputs.internalAngle.`in`(Units.Degrees))
    }
}