package frc.robot

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]

    val CURRENT_MODE: Mode = Mode.SIM

    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }
}
