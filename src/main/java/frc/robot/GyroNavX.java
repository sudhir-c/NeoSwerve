package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class GyroNavX implements GyroIO {
    private AHRS navx;
    public GyroNavX() {
        navx = new AHRS();
        navx.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = navx.getAngle() % 360;
        inputs.pitch = navx.getPitch();
        inputs.roll = navx.getRoll();
        inputs.angularVelocity = navx.getVelocityY();
    }
}
