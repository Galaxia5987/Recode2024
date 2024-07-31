package frc.robot

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.subsystems.swerve.*

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]

    val CURRENT_MODE: Mode = Mode.REAL

    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    fun isRed(): Boolean{
        val alliance = DriverStation.getAlliance()
        return if(alliance.isPresent) alliance.get() == Alliance.Red
        else false
    }

    fun initSwerve() {
        var moduleIOs: Array<ModuleIO>

        when(SwerveConstants.SWERVE_TYPE) {
            SwerveConstants.SwerveType.SIM-> {
                moduleIOs = Array<ModuleIO>(4) { ModuleIOSim() }
                SwerveDrive.initialize(GyroIOSim(), SwerveConstants.OFFSETS, *moduleIOs)
            }
            SwerveConstants.SwerveType.WCP->{
                moduleIOs = Array<ModuleIO>(4) {i->ModuleIOTalonFX(
                    Ports.SwerveDriveWCP.DRIVE_IDS[i],
                    Ports.SwerveDriveWCP.ANGLE_IDS[i],
                    Ports.SwerveDriveWCP.ENCODER_IDS[i],
                    SwerveConstants.DRIVE_MOTOR_CONFIGS ?: TalonFXConfiguration(), //TODO: Not sure if this is great cuz default config doesn't have current limits but it shouldn't be null so...
                    SwerveConstants.ANGLE_MOTOR_CONFIGS ?: TalonFXConfiguration(),
                    SwerveConstants.ENCODER_CONFIGS ?: CANcoderConfiguration(),
                    SwerveConstants.OFFSETS[i]
                )}
            }
            SwerveConstants.SwerveType.NEO->{
                moduleIOs = Array<ModuleIO>(4) {i->ModuleIOSparkMax(
                    Ports.SwerveDriveNEO.DRIVE_IDS[i],
                    Ports.SwerveDriveNEO.ANGLE_IDS[i],
                    Ports.SwerveDriveNEO.ENCODER_IDS[i],
                    Ports.SwerveDriveNEO.DRIVE_INVERTED[i],
                    Ports.SwerveDriveNEO.ANGLE_INVERTED[i]
                )}
            }
        }
    }
}
