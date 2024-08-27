package frc.robot.subsystems.intake

interface IntakeIO {
    fun setSpinPower(power:Double) {}
    fun setCenterPower(power: Double) {}
    fun setAnglePower(power: Double) {}
    fun setAngle(angle:Double) {}
    fun reset() {}
}