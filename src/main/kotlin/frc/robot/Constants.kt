package frc.robot

import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.subsystems.swerve.SwerveConstants
import kotlin.math.sqrt

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]
    const val LOOP_TIME = 0.02 // [s]
    const val IS_TUNING_MODE = true

    private val EFFECTIVE_ROBOT_RADIUS: Measure<Distance> = Units.Meters.of(SwerveConstants.ROBOT_LENGTH / sqrt(2.0))
    private val MAX_VELOCITY: Measure<Velocity<Distance>> = Units.MetersPerSecond.of(2.0)
    private val MAX_ACCELERATION: Measure<Velocity<Velocity<Distance>>> = Units.MetersPerSecondPerSecond.of(1.0)
    private val MAX_ANGULAR_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(
        MAX_VELOCITY.`in`(Units.MetersPerSecond) / EFFECTIVE_ROBOT_RADIUS.`in`(Units.Meters)
    )
    private val MAX_ANGULAR_ACCELERATION: Measure<Velocity<Velocity<Angle>>> =
        Units.RotationsPerSecond.per(Units.Second)
            .of(
                MAX_ACCELERATION.`in`(Units.MetersPerSecondPerSecond) / EFFECTIVE_ROBOT_RADIUS.`in`(Units.Meters)
            )
    val PATH_CONSTRAINTS: PathConstraints = PathConstraints(
        MAX_VELOCITY.`in`(Units.MetersPerSecond),
        MAX_ACCELERATION.`in`(Units.MetersPerSecondPerSecond),
        MAX_ANGULAR_VELOCITY.`in`(Units.RotationsPerSecond),
        MAX_ANGULAR_ACCELERATION.`in`(Units.RotationsPerSecond.per(Units.Second))
    )

    private val SPEAKER_POSE_BLUE = Translation2d(0.0, 5.5479442)

    val SPEAKER_POSE: Translation2d by lazy { if (isRed) GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE) else SPEAKER_POSE_BLUE }

    private val CHAIN_LOCATIONS_BLUE: Array<Pose2d> = arrayOf(
        Triple(4.39, 4.67, -57.72), // Left
        Triple(5.59, 4.09, 180.00), // Center
        Triple(4.39, 3.46, 57.72) // Right
    ).map { (x, y, theta) -> Pose2d(x, y, Rotation2d.fromDegrees(theta)) }.toTypedArray()

    val CHAIN_LOCATIONS: Array<Pose2d>
        get() = if (isRed) Array<Pose2d>(CHAIN_LOCATIONS_BLUE.size)
        { i -> GeometryUtil.flipFieldPose(CHAIN_LOCATIONS_BLUE[i]) } else CHAIN_LOCATIONS_BLUE

    val CURRENT_MODE: Mode = Mode.REAL
    const val ROBORIO_NEO_SERIAL = "030e2d4d"

    val ROBORIO_SERIAL_NUM: String = System.getenv("serialnum") ?: "Sim"

    val isRed: Boolean
        get() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red

    private const val FIELD_LENGTH: Double = 16.54
    private const val FIELD_WIDTH: Double = 8.23

    fun isOutOfBounds(estimatedPose: Pose3d): Boolean {
        val x = estimatedPose.x
        val y = estimatedPose.y
        return !(0 < x && x < FIELD_LENGTH) || !(0 < y && y < FIELD_WIDTH)
    }

    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }
}
