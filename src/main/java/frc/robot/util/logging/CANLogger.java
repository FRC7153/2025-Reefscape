package frc.robot.util.logging;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Logs multiple CAN busses twice a second in the background
 */
public class CANLogger {
  private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

  private final CANBus[] busses;
  private final IntegerLogEntry[] logs;

  public CANLogger(CANBus... busses) {
    this.busses = busses;

    // Init log entries
    logs = new IntegerLogEntry[busses.length * 4];

    for (int b = 0; b < busses.length; b++) {
      String title = "CAN/" + busses[b].getName() + "/";
      logs[b * 4 + 0] = new IntegerLogEntry(DataLogManager.getLog(), title + "Utilization");
      logs[b * 4 + 1] = new IntegerLogEntry(DataLogManager.getLog(), title + "TEC"); // transmission error count
      logs[b * 4 + 2] = new IntegerLogEntry(DataLogManager.getLog(), title + "REC"); // receive error count
      logs[b * 4 + 3] = new IntegerLogEntry(DataLogManager.getLog(), title + "TX_Full");
    }
  }

  public void start() {
    scheduler.scheduleAtFixedRate(this::log, 0, 500, TimeUnit.MILLISECONDS);
  }

  private void log() {
    // Log each CAN bus
    for (int b = 0; b < busses.length; b++) {
      CANBusStatus status = busses[b].getStatus();

      logs[b * 4 + 0].append((int)(status.BusUtilization * 100));
      logs[b * 4 + 1].append(status.TEC);
      logs[b * 4 + 2].append(status.REC);
      logs[b * 4 + 3].append(status.TxFullCount);
    }
  }
}
