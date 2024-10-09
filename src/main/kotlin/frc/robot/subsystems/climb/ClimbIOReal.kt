package frc.robot.subsystems.climb

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Ports

class ClimbIOReal : ClimbIO {
    override val inputs = LoggedClimbInputs()

    private val mainClimbMotor: TalonFX = TalonFX(Ports.Climb.MAIN_MOTOR_ID)
    private val auxClimbMotor: TalonFX = TalonFX(Ports.Climb.AUX_MOTOR_ID)
    private val lockMotor: TalonSRX = TalonSRX(Ports.Climb.STOPPER_ID)

    init {
        mainClimbMotor.configurator.apply(ClimbConstants.MOTOR_CONFIG)
        auxClimbMotor.configurator.apply(ClimbConstants.MOTOR_CONFIG)
    }

    override fun updateInput() {
        inputs.climbMotorVoltage = mainClimbMotor.supplyVoltage.value
    }


    override fun setPower(power: Double) {
        mainClimbMotor.set(power)
        auxClimbMotor.set(power)
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