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

    var SPEAKER_POSE: Translation2d = Translation2d(0.0, 5.5479442) // Blue

    var chainLocations = arrayOf(Pose2d(), Pose2d(), Pose2d()) //left, right, middle

    val CURRENT_MODE: Mode = Mode.REAL

    var CURRENT_STATE: ScoreState? = null

    val isRed: Boolean
        get() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == DriverStation.Alliance.Red


    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    fun initConstants() {
        if (isRed) {
            SPEAKER_POSE = GeometryUtil.flipFieldPosition(SPEAKER_POSE)

            chainLocations = Array<Pose2d>(chainLocations.size)
            { i -> GeometryUtil.flipFieldPose(chainLocations[i])}
        }
    }
}
