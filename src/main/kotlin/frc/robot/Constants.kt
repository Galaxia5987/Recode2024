package frc.robot

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Translation2d
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.units.*
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.scoreState.ScoreState
import frc.robot.subsystems.swerve.*
import kotlin.math.sqrt

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]
    const val LOOP_TIME = 0.02 // [s]

    val MAX_VELOCITY: Measure<Velocity<Distance>> = Units.MetersPerSecond.of(2.0)
    val MAX_ACCELERATION: Measure<Velocity<Velocity<Distance>>> = Units.MetersPerSecondPerSecond.of(1.0)
    val MAX_ANGULAR_VELOCITY: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(
        MAX_VELOCITY.`in`(Units.MetersPerSecond)
                / (SwerveConstants.ROBOT_LENGTH / sqrt(2.0))
    )
    val MAX_ANGULAR_ACCELERATION: Measure<Velocity<Velocity<Angle>>> = Units.RotationsPerSecond.per(Units.Second)
        .of(
            (MAX_ACCELERATION.`in`(Units.MetersPerSecondPerSecond)
                    / (SwerveConstants.ROBOT_LENGTH / sqrt(2.0)))
        )
    val PATH_CONSTRAINTS: PathConstraints = PathConstraints(
        MAX_VELOCITY.`in`(Units.MetersPerSecond),
        MAX_ACCELERATION.`in`(Units.MetersPerSecondPerSecond),
        MAX_ANGULAR_VELOCITY.`in`(Units.RotationsPerSecond),
        MAX_ANGULAR_ACCELERATION.`in`(Units.RotationsPerSecond.per(Units.Second))
    )

    private val SPEAKER_POSE_BLUE = Translation2d(0.0, 5.5479442)

    val SPEAKER_POSE: Translation2d
        get() = if (isRed) GeometryUtil.flipFieldPosition(SPEAKER_POSE_BLUE) else SPEAKER_POSE_BLUE

    private val CHAIN_LOCATIONS_BLUE = arrayOf(Pose2d(), Pose2d(), Pose2d()) // left, right, middle
    val CHAIN_LOCATIONS: Array<Pose2d>
        get() = if (isRed) Array<Pose2d>(CHAIN_LOCATIONS_BLUE.size)
        { i -> GeometryUtil.flipFieldPose(CHAIN_LOCATIONS_BLUE[i])} else CHAIN_LOCATIONS_BLUE

    val CURRENT_MODE: Mode = Mode.REAL

    var CURRENT_STATE: ScoreState? = null

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
