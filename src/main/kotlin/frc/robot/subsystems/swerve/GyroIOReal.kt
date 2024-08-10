package frc.robot.subsystems.swerve

import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.SPI

class GyroIOReal : GyroIO {
    private val gyro = AHRS(SPI.Port.kMXP)
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

    override fun resetGyro(angle: Rotation2d) {
        gyroOffset = rawYaw.minus(angle)
    }

    override fun updateInputs(inputs: SwerveDriveInputs) {
        inputs.gyroOffset = gyroOffset
        inputs.acceleration = gyro.worldLinearAccelX.toDouble() // TODO: Make sure it's really x
        inputs.yaw = yaw
        inputs.rawYaw = rawYaw
    }
}
