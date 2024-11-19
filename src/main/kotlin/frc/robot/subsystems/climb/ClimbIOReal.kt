package frc.robot.subsystems.climb

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Ports

class ClimbIOReal : ClimbIO {
    override val inputs = LoggedClimbInputs()

    private val mainMotor: TalonFX = TalonFX(Ports.Climb.MAIN_MOTOR_ID)
    private val auxMotor: TalonFX = TalonFX(Ports.Climb.AUX_MOTOR_ID)
    private val lockMotor: TalonSRX = TalonSRX(Ports.Climb.STOPPER_ID)

    init {
        mainMotor.configurator.apply(ClimbConstants.MOTOR_CONFIG)
        auxMotor.configurator.apply(ClimbConstants.MOTOR_CONFIG)
    }

    override fun updateInput() {
        inputs.climbMotorVoltage = mainMotor.supplyVoltage.value
        inputs.lockMotorCurrent = lockMotor.supplyCurrent
    }


    override fun setPower(power: Double) {
        mainMotor.set(power)
        auxMotor.set(power)
    }

    override fun lockClimb() {
        lockMotor.set(TalonSRXControlMode.PercentOutput, ClimbConstants.STOPPER_MOTOR_POWER)
    }

    override fun unlockClimb() {
        lockMotor.set(TalonSRXControlMode.PercentOutput, -ClimbConstants.STOPPER_MOTOR_POWER)
    }

    override fun disableLockMotor() {
        lockMotor.neutralOutput()
    }

}