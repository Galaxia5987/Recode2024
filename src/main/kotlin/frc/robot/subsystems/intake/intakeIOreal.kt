package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import frc.robot.Ports

class intakeIOreal() : IntakeIO {

    override val inputs: LoggedIntakeInputs = LoggedIntakeInputs()

    private val rollerMotor = CANSparkMax(Ports.Intake.SPIN_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    private val centerMotor=CANSparkMax(Ports.Intake.CENTER_MOTOR_ID,CANSparkLowLevel.MotorType.kBrushless)

    private val angleMotor=TalonFX(Ports.Intake.ANGLE_MOTOR_ID)

    init {

        val angleMotorConfig=TalonFXConfiguration()
            .withMotorOutput(MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80.0).withSupplyCurrentLimit(40.0)
            )
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(IntakeConstants.gearRatio))
            .withSlot0(Slot0Configs().withKP(IntakeConstants.kp).withKI(IntakeConstants.ki).withKD(IntakeConstants.kd))
        rollerMotor.restoreFactoryDefaults()
        rollerMotor.inverted=true
        rollerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        rollerMotor.enableVoltageCompensation(12.0)
        rollerMotor.setSmartCurrentLimit(40)
        rollerMotor.burnFlash()

        centerMotor.restoreFactoryDefaults()
        centerMotor.inverted=true
        centerMotor.setIdleMode(CANSparkBase.IdleMode.kCoast)
        centerMotor.enableVoltageCompensation(12.0)
        centerMotor.setSmartCurrentLimit(40)
        centerMotor.burnFlash()

    }

    override fun setCenterPower(power: Double) {

    }

}