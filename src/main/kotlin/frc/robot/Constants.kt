package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance

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
}
