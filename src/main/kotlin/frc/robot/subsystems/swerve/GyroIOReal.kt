package frc.robot.subsystems.swerve

import com.studica.frc.AHRS
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.pow
import kotlin.math.sqrt

class GyroIOReal : GyroIO {
    private val gyro = AHRS(AHRS.NavXComType.kMXP_SPI)
    private var gyroOffset = Rotation2d()

    init {
        gyro.reset()
    }

    val yaw: Rotation2d
        get() = rawYaw.minus(gyroOffset)

    val rawYaw: Rotation2d
        get() = Rotation2d(-MathUtil.angleModulus(Math.toRadians(gyro.angle)))

    val pitch: Rotation2d
        get() = Rotation2d(gyro.pitch.toDouble())

    override fun getAccelMagnitude(): Float {
        return sqrt(gyro.rawAccelX.pow(2) + gyro.rawAccelY.pow(2) + gyro.rawAccelZ.pow(2))
    }

    override fun resetGyro(angle: Rotation2d) {
        gyroOffset = rawYaw.minus(angle)
    }

    override fun updateInputs(inputs: SwerveDriveInputs) {
        inputs.gyroOffset = gyroOffset
        inputs.acceleration = getAccelMagnitude()
        inputs.yaw = yaw
        inputs.rawYaw = rawYaw
    }
}
