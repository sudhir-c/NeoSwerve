package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import javax.sound.sampled.Port;

public class GyroNavX implements GyroIO {
    private AHRS navx;
    public GyroNavX() {
        navx = new AHRS(SPI.Port.kOnboardCS0);
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
