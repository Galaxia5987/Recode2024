package frc.robot

object Constants {
    const val CONFIG_TIMEOUT: Int = 100 // [ms]
    const val LOOP_TIME = 0.02 // [s]

    val CURRENT_MODE: Mode = Mode.SIM


    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }
}
