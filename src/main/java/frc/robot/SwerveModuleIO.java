package frc.robot;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    default void updateInputs(SwerveModuleIOInputs swerveModuleIOInputs) {}
    default void setTargetSteerPosition(double targetSteerPosition) {}
    default void setTargetDriveVelocity(double targetDriveVelocity) {}
    default double getMaxVelocity() {return 0.0;}
    @AutoLog
    class SwerveModuleIOInputs {
        public double drivePositionMeters;
        public double steerPositionRad;
    }

}
