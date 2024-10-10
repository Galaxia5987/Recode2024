package frc.robot.subsystems.shooter

import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.LoggedTunableNumber
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Shooter private constructor(private val io: ShooterIO) : SubsystemBase() {
    private val topKP =
        LoggedTunableNumber("Shooter/Top kP", ShooterConstants.TOP_GAINS.kP)
    private val topKI =
        LoggedTunableNumber("Shooter/Top kI", ShooterConstants.TOP_GAINS.kI)
    private val topKD =
        LoggedTunableNumber("Shooter/Top kD", ShooterConstants.TOP_GAINS.kD)
    private val topKS =
        LoggedTunableNumber("Shooter/Top kS", ShooterConstants.TOP_GAINS.kS)
    private val topKV =
        LoggedTunableNumber("Shooter/Top kV", ShooterConstants.TOP_GAINS.kV)
    private val topKA =
        LoggedTunableNumber("Shooter/Top kA", ShooterConstants.TOP_GAINS.kA)

    private val bottomKP =
        LoggedTunableNumber("Shooter/Bottom kP", ShooterConstants.BOTTOM_GAINS.kP)
    private val bottomKI =
        LoggedTunableNumber("Shooter/Bottom kI", ShooterConstants.BOTTOM_GAINS.kI)
    private val bottomKD =
        LoggedTunableNumber("Shooter/Bottom kD", ShooterConstants.BOTTOM_GAINS.kD)
    private val bottomKS =
        LoggedTunableNumber("Shooter/Bottom kS", ShooterConstants.BOTTOM_GAINS.kS)
    private val bottomKV =
        LoggedTunableNumber("Shooter/Bottom kV", ShooterConstants.BOTTOM_GAINS.kV)
    private val bottomKA =
        LoggedTunableNumber("Shooter/Bottom kA", ShooterConstants.BOTTOM_GAINS.kA)

    @AutoLogOutput
    private var topVelocitySetpoint: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()

    @AutoLogOutput
    private var bottomVelocitySetpoint: Measure<Velocity<Angle>> = Units.RotationsPerSecond.zero()
    private val timer = Timer()
    private val subsystemName = this::class.simpleName

    companion object {
        @Volatile
        private var instance: Shooter? = null

        fun initialize(io: ShooterIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Shooter(io)
                }
            }
        }

        fun getInstance(): Shooter {
            return instance ?: throw IllegalArgumentException(
                "Shooter has not been initialized. Call initialize(io: ShooterIO) first."
            )
        }
    }

    init {
        timer.start()
        timer.reset()
    }

    fun setVelocity(topVelocity: Measure<Velocity<Angle>>, bottomVelocity: Measure<Velocity<Angle>>): Command {
        return runOnce {
            topVelocitySetpoint = topVelocity
            bottomVelocitySetpoint = bottomVelocity
            io.setTopVelocity(topVelocity)
            io.setBottomVelocity(bottomVelocity)
        }.withName("Set Top and Bottom Velocity Command")
    }

    fun setVelocity(velocitySupplier: () -> Measure<Velocity<Angle>>): Command {
        return run {
            val velocity = velocitySupplier.invoke()
            topVelocitySetpoint = velocity
            bottomVelocitySetpoint = velocity
            io.setTopVelocity(velocity)
            io.setBottomVelocity(velocity)
        }.withName("Set Top and Bottom Velocity Command")
    }

    fun setVelocity(velocity: Measure<Velocity<Angle>>): Command =
        setVelocity(velocity, velocity).withName("Set Velocity Command")

    fun stop(): Command {
        return run {
            topVelocitySetpoint = ShooterConstants.STOP_POWER
            bottomVelocitySetpoint = ShooterConstants.STOP_POWER
            io.stop()
        }.withName("Stop Shooter")
    }

    fun rollNote(): Command {
        return setVelocity(Units.RotationsPerSecond.of(-5.0), Units.RotationsPerSecond.of(5.0))
    }

    @AutoLogOutput
    fun atSetpoint(): Boolean =
        io.topRollerInputs.velocity.isNear(
            topVelocitySetpoint, ShooterConstants.TOP_ROLLER_TOLERANCE.`in`(Units.Percent)
        ) && io.bottomRollerInputs.velocity.isNear(
            bottomVelocitySetpoint, ShooterConstants.BOTTOM_ROLLER_TOLERANCE.`in`(
                Units.Percent
            )
        )

    override fun periodic() {
        LoggedTunableNumber.ifChanged(
            hashCode(), { kPIDSVA: DoubleArray ->
                io.setTopGains(
                    kPIDSVA[0], kPIDSVA[1], kPIDSVA[2], kPIDSVA[3], kPIDSVA[4], kPIDSVA[5]
                )
            }, topKP, topKI, topKD, topKS, topKV, topKA
        )
        LoggedTunableNumber.ifChanged(
            hashCode(), { kPIDSVA: DoubleArray ->
                io.setBottomGains(
                    kPIDSVA[0], kPIDSVA[1], kPIDSVA[2], kPIDSVA[3], kPIDSVA[4], kPIDSVA[5]
                )
            }, bottomKP, bottomKI, bottomKD, bottomKS, bottomKV, bottomKA
        )

        io.updateInputs()
        Logger.processInputs("$subsystemName/TopRoller", io.topRollerInputs)
        Logger.processInputs("$subsystemName/BottomRoller", io.bottomRollerInputs)
    }
}