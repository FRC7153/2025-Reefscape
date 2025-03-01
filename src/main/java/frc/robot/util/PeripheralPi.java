package frc.robot.util;

import java.util.BitSet;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;

public class PeripheralPi {
  private final CAN pi = new CAN(20, 8, 11);

  private final Thread thread = new Thread(this::refresh);

  public PeripheralPi() {
    thread.setDaemon(true);
    thread.start();
  }

  private void refresh() {
    CANData canData = new CANData();

    while (true) { 
        if (pi.readPacketNew(0b0000010000, canData)) {
          BitSet data = BitSet.valueOf(canData.data);

          // Reverse each byte individually
          for (int x = 0; x < 8; x++) {
            boolean[] newByte = new boolean[8];

            for (int b = 0; b < 8; b++) {
              newByte[b] = data.get((x*8) + (7 - b));
            }

            for (int b = 0; b < 8; b++) {
              data.set((x*8) + b, newByte[b]);
            }
          }

          
        }
    }
  }
}
