package frc.robot.subsystems.TLArm

class TLArm private constructor(private var io: TLArmIO) {
    private var inputs = io.inputs

    companion object {
        @Volatile
        private var instance: TLArm? = null

        fun initialize(io: TLArmIO) {
            synchronized(true) {
                if (instance == null) {
                    instance = TLArm(io)
                }
            }
        }

        fun getInstance(): TLArm = instance ?: throw IllegalStateException(
            "telescopic arm has not been initialized. Call initialize(io:TLArmIO) first"
        )

    }
}