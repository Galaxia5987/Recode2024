package frc.robot.subsystems.TLArm

import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Ports

class TLArmIOReal : TLArmIO {
    override var inputs = LoggedTLArmInputs()
    var motor: TalonFX = TalonFX(Ports.TLArm.TL_MOTOR_ID)
    override fun updateInput() {
        inputs.currentPose = motor.position.value
    }

    override fun setPosition(setPoint: Double) {
        motor.setPosition(setPoint)
    }
}