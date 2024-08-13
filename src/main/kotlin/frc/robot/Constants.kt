package frc.robot

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.swerve.*

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]
    const val LOOP_TIME = 0.02 // [s]

    val CURRENT_MODE: Mode = Mode.REAL

    var SPEAKER_POSE: Translation2d = Translation2d(0.0, 5.5479442) // Blue

    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    fun isRed(): Boolean {
        val alliance = DriverStation.getAlliance()
        return if (alliance.isPresent) alliance.get() == Alliance.Red
        else false
    }

    fun initSpeakerPose() {
        if (isRed()) {
            SPEAKER_POSE = GeometryUtil.flipFieldPosition(SPEAKER_POSE)
        }
    }

    fun initSwerve() {
        var moduleIOs: Array<ModuleIO>

        when (SwerveConstants.SWERVE_TYPE) {
            SwerveConstants.SwerveType.SIM -> {
                moduleIOs = Array<ModuleIO>(4) { ModuleIOSim() }
                SwerveDrive.initialize(GyroIOSim(), SwerveConstants.OFFSETS, *moduleIOs)
            }

            SwerveConstants.SwerveType.WCP -> {
                moduleIOs = Array<ModuleIO>(4) { i ->
                    ModuleIOTalonFX(
                        Ports.SwerveDriveWCP.DRIVE_IDS[i],
                        Ports.SwerveDriveWCP.ANGLE_IDS[i],
                        Ports.SwerveDriveWCP.ENCODER_IDS[i],
                        SwerveConstants.DRIVE_MOTOR_CONFIGS
                            ?: throw IllegalStateException("drive motor config is null"),
                        SwerveConstants.ANGLE_MOTOR_CONFIGS
                            ?: throw IllegalStateException("angle motor config is null"),
                        SwerveConstants.ENCODER_CONFIGS ?: throw IllegalStateException("encoder config is null")
                    )
                }
                SwerveDrive.initialize(GyroIOReal(), SwerveConstants.OFFSETS, *moduleIOs)
            }

            SwerveConstants.SwerveType.NEO -> {
                moduleIOs = Array<ModuleIO>(4) { i ->
                    ModuleIOSparkMax(
                        Ports.SwerveDriveNEO.DRIVE_IDS[i],
                        Ports.SwerveDriveNEO.ANGLE_IDS[i],
                        Ports.SwerveDriveNEO.ENCODER_IDS[i],
                        Ports.SwerveDriveNEO.DRIVE_INVERTED[i],
                        Ports.SwerveDriveNEO.ANGLE_INVERTED[i]
                    )
                }
                SwerveDrive.initialize(GyroIOReal(), SwerveConstants.OFFSETS, *moduleIOs)
            }
        }

    }
}
