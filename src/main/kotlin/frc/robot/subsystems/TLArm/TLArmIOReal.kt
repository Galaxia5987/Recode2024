package frc.robot.subsystems.TLArm

import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import frc.robot.Ports

class TLArmIOReal:TLArmIO {
    override var inputs = LoggedTLArmInputs()
    var tlMotor:TalonFX = TalonFX(Ports.TLArm.TL_MOTOR_ID)
    override fun updateInput() {
        inputs.currentPose = tlMotor.position.value
    }

    override fun setPosition(setPoint: Double) {
        tlMotor.setPosition(setPoint)
    }
}