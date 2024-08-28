package frc.robot.lib

data class Gains(
    private val _kP: Double? = null,
    private val _kI: Double? = null,
    private val _kD: Double? = null,
    private val _kS: Double? = null,
    private val _kV: Double? = null,
    private val _kA: Double? = null,
    private val _kG: Double? = null
) {
    val kP: Double
        get() = _kP ?: throw IllegalStateException("kP has not been initialized")

    val kI: Double
        get() = _kI ?: throw IllegalStateException("kI has not been initialized")

    val kD: Double
        get() = _kD ?: throw IllegalStateException("kD has not been initialized")

    val kS: Double
        get() = _kS ?: throw IllegalStateException("kS has not been initialized")

    val kV: Double
        get() = _kV ?: throw IllegalStateException("kV has not been initialized")

    val kA: Double
        get() = _kA ?: throw IllegalStateException("kA has not been initialized")

    val kG: Double
        get() = _kG ?: throw IllegalStateException("kG has not been initialized")
}
