"""1 Hz power log for finding why the Pi switches off. Each line is fsynced,
so the last seconds before a power cut survive. Delete when solved.
Columns: time, EXT5V volts, throttled flags, temperature, 1-min load."""
import os
import subprocess
import time

PATH = "/home/caconee/power_log.csv"


def run(*args):
    return subprocess.run(args, capture_output=True, text=True).stdout.strip()


with open(PATH, "a") as f:
    f.write("# boot %s\n" % time.strftime("%F %T"))
    while True:
        volt = run("vcgencmd", "pmic_read_adc", "EXT5V_V").split("=")[-1]
        thr = run("vcgencmd", "get_throttled").split("=")[-1]
        temp = run("vcgencmd", "measure_temp").split("=")[-1]
        with open("/proc/loadavg") as loadavg:
            load = loadavg.read().split()[0]
        f.write(",".join((time.strftime("%T"), volt, thr, temp, load)) + "\n")
        f.flush()
        os.fsync(f.fileno())
        time.sleep(1)
