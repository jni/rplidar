"""Play a chime when something comes too close to the lidar."""
import numpy as np
import matplotlib.pyplot as plt
import chime

from rplidar import RPLidar, RPLidarException

chime.theme('mario')
lidar = RPLidar('/dev/tty.usbserial-0001')
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
scan_data = np.zeros((2000, 4), dtype=float)

try:
    i = 0
    for scan in lidar.iter_scans():
        for s in scan:
            quality, angle_deg, distance_mm = s
            scan_data[i % scan_data.shape[0]] = (i,) + s
            i += 1
            if (0 <= angle_deg <= 30
                    or 330 <= angle_deg <= 360) and distance_mm < 2100:
                chime.info()
except RPLidarException as exc:
    lidar.logger.debug(str(exc))
except KeyboardInterrupt:
    lidar.logger.debug('stopped')
finally:
    lidar.logger.debug('finally')
    lidar.reset()
    lidar.stop()
    lidar.stop_motor()

ax.set_theta_direction(-1)
ax.set_theta_zero_location('N')
ax.scatter(np.deg2rad(scan_data[:, 2]), scan_data[:, 3]/1000)
plt.show()
