import csv
import time
from datetime import datetime
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt

# --- Configuration ---
BAUD_RATE = 115200
CSV_FILENAME = f"dropsonde_profile_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

def select_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise RuntimeError("No serial ports found. Plug in the ground ESP32-C3.")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
    idx = int(input("Select ground station port index: "))
    return ports[idx].device

def main():
    port_name = select_serial_port()
    ser = serial.Serial(port_name, BAUD_RATE, timeout=0.2)
    time.sleep(1.5)

    # Data buffers for live plotting
    alts, temps, presses, hums, wind_spds = [], [], [], [], []

    # Setup 2x2 Atmospheric Vertical Profile Plot
    plt.ion()
    fig, axs = plt.subplots(2, 2, figsize=(11, 8), sharey=True)
    fig.suptitle("Dropsonde Live Atmospheric Profile", fontsize=14, fontweight="bold")

    ax_temp, ax_press = axs[0, 0], axs[0, 1]
    ax_hum, ax_wind   = axs[1, 0], axs[1, 1]

    ax_temp.set_xlabel("Temperature (°C)")
    ax_temp.set_ylabel("Barometric Altitude (m)")
    ax_press.set_xlabel("Pressure (hPa)")
    ax_hum.set_xlabel("Relative Humidity (%)")
    ax_hum.set_ylabel("Barometric Altitude (m)")
    ax_wind.set_xlabel("Horizontal Wind Speed (m/s)")

    for ax in axs.flat:
        ax.grid(True, linestyle="--", alpha=0.5)

    line_temp,  = ax_temp.plot([], [], "r-o", markersize=3)
    line_press, = ax_press.plot([], [], "b-o", markersize=3)
    line_hum,   = ax_hum.plot([], [], "g-o", markersize=3)
    line_wind,  = ax_wind.plot([], [], "m-o", markersize=3)

    headers = [
        "pc_timestamp", "packet_id", "fix_valid", "lat", "lon",
        "gps_alt_m", "baro_alt_m", "wind_spd_ms", "wind_dir_deg",
        "press_hpa", "temp_c", "hum_pct", "sats", "rssi_dbm", "snr_db"
    ]

    print(f"Logging telemetry to {CSV_FILENAME}... Press Ctrl+C to stop.")

    with open(CSV_FILENAME, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)

        try:
            while True:
                raw_line = ser.readline().decode("utf-8", errors="ignore").strip()
                if raw_line.startswith("DATA:"):
                    parts = raw_line.replace("DATA:", "").split(",")
                    if len(parts) == 14:
                        now_str = datetime.now().isoformat(timespec="milliseconds")
                        writer.writerow([now_str] + parts)
                        f.flush()

                        pkt_id    = int(parts[0])
                        gps_alt   = float(parts[4])
                        baro_alt  = float(parts[5])
                        wind_spd  = float(parts[6])
                        wind_dir  = float(parts[7])
                        press_hpa = float(parts[8])
                        temp_c    = float(parts[9])
                        hum_pct   = float(parts[10])
                        sats      = int(parts[11])
                        rssi      = float(parts[12])

                        # Use barometric altitude for smooth vertical profiling
                        alts.append(baro_alt)
                        temps.append(temp_c)
                        presses.append(press_hpa)
                        hums.append(hum_pct)
                        wind_spds.append(wind_spd)

                        print(
                            f"PKT #{pkt_id:04d} | Alt: {baro_alt:7.1f}m (GPS:{gps_alt:6.1f}m) | "
                            f"T: {temp_c:5.2f}°C | P: {press_hpa:6.1f}hPa | RH: {hum_pct:4.1f}% | "
                            f"Wind: {wind_spd:4.1f}m/s @ {wind_dir:5.1f}° | Sats:{sats} | RSSI:{rssi}dBm"
                        )

                        # Update plots
                        line_temp.set_data(temps, alts)
                        line_press.set_data(presses, alts)
                        line_hum.set_data(hums, alts)
                        line_wind.set_data(wind_spds, alts)

                        for ax in axs.flat:
                            ax.relim()
                            ax.autoscale_view()

                        plt.pause(0.01)

                plt.pause(0.01)

        except KeyboardInterrupt:
            print(f"\nSession ended. Saved {len(alts)} records to {CSV_FILENAME}.")
        finally:
            ser.close()
            plt.ioff()
            plt.show()

if __name__ == "__main__":
    main()