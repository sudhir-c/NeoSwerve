package frc.robot;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    default void updateInputs(GyroIOInputs inputs) {}
    default void zeroYaw() {}

    @AutoLog
    class GyroIOInputs {
        public double yaw = 0;
        public double pitch = 0;
        public double roll = 0;
        public double angularVelocity = 0;
    }

}
