package frc.robot.subsystems.intake



interface IntakeIO {
    fun setCenterPower(power: Double)
    fun setRollerPower(power: Double)
    fun setAngleMotorPos(position: Double)
    fun setAngleMotorPow(power: Double)
    fun reset()


}
