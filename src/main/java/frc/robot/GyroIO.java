package frc.robot;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

    default void updateInputs(GyroIOInputs inputs) {}
    default void zeroYaw() {}

    @AutoLog
    class GyroIOInputs {
        public double yaw = 10;
        public double pitch = 10;
        public double roll = 10;
        public double angularVelocity = 10;
    }

}
