package frc.robot.subsystems.swerve

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkClosedLoopController
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.Utils
import frc.robot.lib.units.Units
import edu.wpi.first.units.Units as WpiUnits

class ModuleIOSparkMax(
    driveMotorID: Int,
    angleMotorID: Int,
    encoderID: Int,
    driveInverted: Boolean,
    angleInverted: Boolean
) : ModuleIO {
    private val driveMotor: SparkMax
    private val drivePIDController: SparkClosedLoopController
    private val driveEncoder: RelativeEncoder
    private var driveConfigurator = SparkMaxConfig()
    private val angleMotor: SparkMax
    private val anglePIDController: SparkClosedLoopController
    private val angleEncoder: RelativeEncoder
    private var angleConfigurator = SparkMaxConfig()

    private val encoder: DutyCycleEncoder

    private var feedforward: SimpleMotorFeedforward? = null
    override val inputs = LoggedModuleInputs()

    init {
        this.driveMotor = SparkMax(driveMotorID, SparkLowLevel.MotorType.kBrushless)
        this.angleMotor = SparkMax(angleMotorID, SparkLowLevel.MotorType.kBrushless)

        this.encoder = DutyCycleEncoder(encoderID)

        drivePIDController = driveMotor.closedLoopController
        driveEncoder = driveMotor.encoder

        driveConfigurator = SparkMaxConfig().apply {
            voltageCompensation(SwerveConstants.VOLT_COMP_SATURATION)
            smartCurrentLimit(SwerveConstants.NEO_CURRENT_LIMIT.toInt())
            inverted(driveInverted)
                .encoder.positionConversionFactor(SwerveConstants.DRIVE_REDUCTION)
                .velocityConversionFactor(SwerveConstants.DRIVE_REDUCTION)
        }
        driveMotor.configure(
            driveConfigurator,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )

        anglePIDController = angleMotor.closedLoopController
        angleEncoder = angleMotor.encoder

        angleConfigurator = SparkMaxConfig().apply {
            voltageCompensation(SwerveConstants.VOLT_COMP_SATURATION)
            idleMode(SparkBaseConfig.IdleMode.kBrake)
            smartCurrentLimit(SwerveConstants.NEO_550_CURRENT_LIMIT.toInt())
            inverted(angleInverted)
                .encoder.positionConversionFactor(SwerveConstants.ANGLE_REDUCTION)
                .velocityConversionFactor(SwerveConstants.ANGLE_REDUCTION)
        }
        angleMotor.configure(
            angleConfigurator,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
    }

    override fun updateInputs() {
        inputs.absolutePosition = encoderAngle

        inputs.driveMotorPosition = driveEncoder.position
        inputs.driveMotorVelocity = velocity

        inputs.angle =
            Rotation2d.fromRadians(Utils.normalize(angleEncoder.position * 2 * Math.PI))

        inputs.moduleDistance =
            (
                    inputs.driveMotorPosition
                            * SwerveConstants.WHEEL_DIAMETER
                            * Math.PI
                    )
    }

    override var angle
        get() = inputs.angle
        set(angle) {
            inputs.angleSetpoint = Utils.normalize(angle)
            val error = angle.minus(inputs.angle)
            anglePIDController.setReference(
                inputs.angle.getRotations() + error.rotations,
                SparkBase.ControlType.kPosition
            )
        }

    override var velocity
        get() = (
                Units.rpmToRadsPerSec(driveEncoder.velocity) *
                        (SwerveConstants.WHEEL_DIAMETER / 2)
                )
        set(velocity) {
            var velocity = velocity
            val angleError = inputs.angleSetpoint.minus(inputs.angle)
            velocity *= angleError.cos
            inputs.driveMotorVelocitySetpoint = velocity
            drivePIDController.setReference(
                feedforward!!.calculate(WpiUnits.MetersPerSecond.of(velocity)).`in`(WpiUnits.Volts),
                SparkBase.ControlType.kVoltage
            )
        }

    override val moduleState
        get() = SwerveModuleState(velocity, inputs.angle)

    override val modulePosition
        get() = SwerveModulePosition(inputs.moduleDistance, angle)

    override fun stop() {
        driveMotor.stopMotor()
        angleMotor.stopMotor()
    }

    override fun checkModule(): Command? {
        return Commands.run(
            {
                driveMotor.set(0.8)
                angleMotor.set(0.2)
            })
    }

    override fun updateOffset(offset: Rotation2d) {
        angleEncoder.setPosition(encoderAngle - offset.rotations)
    }

    private val encoderAngle: Double
        get() = 1.0 - encoder.get()

    override fun setIdleMode(isBreakMode: Boolean) {
        val mode = if (isBreakMode) IdleMode.kBrake else IdleMode.kCoast

        mapOf(driveConfigurator to driveMotor, angleConfigurator to angleMotor).forEach {
            it.key.idleMode(mode)
            it.value.configure(
                it.key,
                SparkBase.ResetMode.kNoResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
            )
        }
    }
}
