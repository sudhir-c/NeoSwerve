package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

import javax.sound.sampled.Port;

public class GyroNavX implements GyroIO {
    private AHRS navx;
    public GyroNavX() {
        navx = new AHRS(SerialPort.Port.kUSB);
        navx.calibrate();
        navx.resetDisplacement();
        navx.zeroYaw();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = -1 * navx.getAngle() % 360;
        inputs.pitch = navx.getPitch();
        inputs.roll = navx.getRoll();
        inputs.angularVelocity = navx.getVelocityY();
    }

    @Override
    public void zeroYaw() {
        navx.zeroYaw();
    }
}
