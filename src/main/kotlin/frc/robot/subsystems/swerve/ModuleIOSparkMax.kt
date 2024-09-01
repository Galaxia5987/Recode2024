package frc.robot.subsystems.swerve

import com.revrobotics.*
import edu.wpi.first.math.controller.SimpleMotorFeedforward
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.Utils
import frc.robot.lib.units.Units

class ModuleIOSparkMax(
    driveMotorID: Int,
    angleMotorID: Int,
    encoderID: Int,
    driveInverted: Boolean,
    angleInverted: Boolean
) : ModuleIO {
    private val driveMotor: CANSparkMax
    private val drivePIDController: SparkPIDController
    private val driveEncoder: RelativeEncoder
    private val angleMotor: CANSparkMax
    private val anglePIDController: SparkPIDController
    private val angleEncoder: RelativeEncoder

    private val encoder: DutyCycleEncoder

    private var feedforward: SimpleMotorFeedforward? = null
    override val inputs = LoggedModuleInputs()

    init {
        this.driveMotor = CANSparkMax(driveMotorID, CANSparkLowLevel.MotorType.kBrushless)
        this.angleMotor = CANSparkMax(angleMotorID, CANSparkLowLevel.MotorType.kBrushless)

        this.encoder = DutyCycleEncoder(encoderID)

        driveMotor.restoreFactoryDefaults()
        drivePIDController = driveMotor.pidController
        driveEncoder = driveMotor.encoder

        driveMotor.enableVoltageCompensation(
            SwerveConstants.VOLT_COMP_SATURATION
        )
        driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        driveMotor.setSmartCurrentLimit(
            SwerveConstants.NEO_CURRENT_LIMIT.toInt()
        )
        driveMotor.inverted = driveInverted
        driveEncoder.setPositionConversionFactor(
            SwerveConstants.DRIVE_REDUCTION
        )
        driveEncoder.setVelocityConversionFactor(
            SwerveConstants.DRIVE_REDUCTION
        )
        driveMotor.burnFlash()

        angleMotor.restoreFactoryDefaults()
        anglePIDController = angleMotor.pidController
        angleEncoder = angleMotor.encoder

        angleMotor.enableVoltageCompensation(
            SwerveConstants.VOLT_COMP_SATURATION
        )
        angleMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        angleMotor.setSmartCurrentLimit(
            SwerveConstants.NEO_550_CURRENT_LIMIT.toInt()
        )
        angleMotor.inverted = angleInverted
        angleEncoder.setPositionConversionFactor(
            SwerveConstants.ANGLE_REDUCTION
        )
        angleEncoder.setVelocityConversionFactor(
            SwerveConstants.ANGLE_REDUCTION
        )
        angleMotor.burnFlash()
    }

    override fun updateInputs() {
        inputs.absolutePosition = encoderAngle

        inputs.driveMotorPosition = driveEncoder.position
        inputs.driveMotorVelocity = velocity

        inputs.angle =
            Rotation2d.fromRadians(Utils.normalize(angleEncoder.position * 2 * Math.PI))

        inputs.moduleDistance =
            (inputs.driveMotorPosition
                    * SwerveConstants.WHEEL_DIAMETER
                    * Math.PI)

    }

    override var angle
        get() = inputs.angle
        set(angle) {
            inputs.angleSetpoint = Utils.normalize(angle)
            val error = angle.minus(inputs.angle)
            anglePIDController.setReference(
                inputs.angle.getRotations() + error.rotations,
                CANSparkBase.ControlType.kPosition
            )
        }

    override var velocity
        get() = (Units.rpmToRadsPerSec(driveEncoder.velocity)
                * (SwerveConstants.WHEEL_DIAMETER / 2))
        set(velocity) {
            var velocity = velocity
            val angleError = inputs.angleSetpoint.minus(inputs.angle)
            velocity *= angleError.cos
            inputs.driveMotorVelocitySetpoint = velocity
            drivePIDController.setReference(
                feedforward!!.calculate(velocity), CANSparkBase.ControlType.kVoltage
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
        get() = 1.0 - encoder.absolutePosition
}
