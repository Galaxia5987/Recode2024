package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.GravityTypeValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.*
import frc.robot.lib.webconstants.LoggedTunableNumber

object HoodConstants {
    const val GEAR_RATIO : Double = 3.0 * (36.0 / 18.0) * (158.0 / 18.0)
    const val ENCODER_TICKS_PER_REVOLUTION = 4096.0
    const val SIMULATION_LENGTH = 3.0
    const val MAX_TOLERANCE : Double = 0.75/360

    val MOMENT_OF_INERTIA : Measure<Mult<Mult<Mass, Distance>, Distance>> = Units.Kilograms.mult(Units.Meters).mult(Units.Meters).of(0.0003);
    val MAX_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(1.0)
    val MAX_ACCELERATION: Measure<Velocity<Velocity<Angle>>> = Units.RotationsPerSecond.per(Units.Second).of(4.0)
    val RESTING_ANGLE: Measure<Angle> = Units.Degrees.of(100.0)
    val HOOD_LENGTH: Measure<Distance> = Units.Meters.of(0.4)
   
    val MECHANISM_2D_POSE = Translation2d(1.0, 1.0)
    val ROOT_POSITION = Translation3d(-0.27, 0.0, 0.225)
    val SIMULATION_OFFSET: Measure<Angle> = Units.Degrees.of(-54.0)

    private const val CURRENT_LIMIT = 40.0
    private val INVERTED_VALUE = InvertedValue.CounterClockwise_Positive


    val ABSOLUTE_ENCODER_OFFSET = LoggedTunableNumber("Hood/EncoderOffset")

    val kP = LoggedTunableNumber("Hood/kP")
    val kI = LoggedTunableNumber("Hood/kI")
    val kD = LoggedTunableNumber("Hood/kD")
    val kS = LoggedTunableNumber("Hood/kS")
    val kV = LoggedTunableNumber("Hood/kV")
    val kA = LoggedTunableNumber("Hood/kA")
    val kG = LoggedTunableNumber("Hood/kG")

    val MOTOR_CONFIGURATION = TalonFXConfiguration()

    init {
        kP.initDefault(3.0)
        ABSOLUTE_ENCODER_OFFSET.initDefault((78.046 - 33.48) / 360.0)
        MOTOR_CONFIGURATION
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
            .withMotionMagic(
                MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        MAX_VELOCITY.`in`(Units.RotationsPerSecond)
                    )
                    .withMotionMagicJerk(16.0)
            )
            .withSlot0(
                Slot0Configs()
                    .withKP(kP.get())
                    .withKI(kI.get())
                    .withKD(kD.get())
                    .withKS(kS.get())
                    .withKV(kV.get())
                    .withKA(kA.get())
                    .withKG(kG.get())
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
            )
            .withMotorOutput(MotorOutputConfigs().withInverted(HoodConstants.INVERTED_VALUE)).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(2 * HoodConstants.CURRENT_LIMIT)
            .withSupplyCurrentLimit(HoodConstants.CURRENT_LIMIT)
    }
}