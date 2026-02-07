package frc.robot.hardware;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.AlertManager;

public class PowerManagement {

    private static final int NUM_PDH_CHANNELS = 24;
    private final PowerDistribution powerDistributionHub;
    private final Alert brownoutAlert = new Alert("PDH Brownout detected!", AlertType.kWarning);
    private final Set<Integer> channelsInUse;  //avoid alerting on channels we know are not in active use
    private final Map<Integer, String> channelToDeviceMap; //map of PDH channel IDs to device names for clearer alerting

    //Note: not all powered devices go through the PDH!
    // The following devices are on the VRM and won't be monitored here as there is no API access to telemetry from the VRM:
    // Pigeon2
    public PowerManagement() {
        powerDistributionHub = new PowerDistribution(Constants.CanID.POWER_DISTRIBUTION, ModuleType.kRev);
        channelsInUse = new HashSet<>();
        channelToDeviceMap = new HashMap<>();
        //TODO: Keep this up to date as new devices are added or removed
        channelsInUse.add(0);
        channelToDeviceMap.put(0, "Back Right Steer");
        channelsInUse.add(1);
        channelToDeviceMap.put(1, "Back Right Drive");
        channelsInUse.add(2);
        channelToDeviceMap.put(2, "Front Right Steer");
        channelsInUse.add(3);
        channelToDeviceMap.put(3, "Front Right Drive");
        channelsInUse.add(4);
        channelToDeviceMap.put(4, "Left Spindexer NEO");
        channelsInUse.add(5);
        channelToDeviceMap.put(5, "Right Spindexer NEO");
        channelsInUse.add(6);
        channelToDeviceMap.put(6, "Front Left Steer");
        channelsInUse.add(7);
        channelToDeviceMap.put(7, "Front Left Drive");
        channelsInUse.add(8);
        channelToDeviceMap.put(8, "Back Left Steer");
        channelsInUse.add(9);
        channelToDeviceMap.put(9, "Back Left Drive");
    }

    public void updateTelemetry() {
        SmartDashboard.putNumber("PDH Input Voltage", powerDistributionHub.getVoltage());
        SmartDashboard.putNumber("PDH Total Current", powerDistributionHub.getTotalCurrent());

        if (powerDistributionHub.getFaults().Brownout) {
            brownoutAlert.set(true);
        }

        for (int channel = 0; channel < NUM_PDH_CHANNELS; channel++) {
            if (powerDistributionHub.getFaults().getBreakerFault(channel) && channelsInUse.contains(channel)) {
                AlertManager.addAlert("PDH-Channel-" + channel, "PDH Breaker fault detected on channel " + channel + " (" + channelToDeviceMap.get(channel) + ") - current = " + powerDistributionHub.getCurrent(channel), AlertType.kWarning);
            } else {
                AlertManager.removeAlert("PDH-Channel-" + channel);
            }
        }
    }
}
