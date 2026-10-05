"""Ground-truth measurement field from real Mars Odyssey GRS water-concentration data.

Used for taking measurements and for evaluation only. The adaptive sampler
must only ever call measure(); ground_truth() is for scoring afterwards.

Owner: Karan Sharma
Task: Advanced 6

Data: 2001 Mars Odyssey Gamma Ray Spectrometer, H2O concentration [weight %],
smoothed 5x5 degree map (NASA PDS data set ODY-M-GRS-5-ELEMENTS-V1.0).
See data/mars_odyssey_grs/README.md.
"""

import os

import numpy as np
from scipy.interpolate import RegularGridInterpolator

NOT_APPLICABLE = 9999.0

# Tharsis region centred near Arsia Mons (8.4 S, 239.9 E), where lava-tube
# skylights have been observed. Bin centres, degrees, east longitude.
DEFAULT_LAT_RANGE = (-37.5, 17.5)
DEFAULT_LON_RANGE = (212.5, 267.5)


def default_table_path():
    """Path of the installed GRS water table"""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('cave_explorer'),
                        'data', 'mars_odyssey_grs', 'h2o_sr_5x5.tab')


class MeasurementField:
    def __init__(self, table_path, x_range, y_range,
                 lat_range=DEFAULT_LAT_RANGE, lon_range=DEFAULT_LON_RANGE,
                 noise_scale=1.0, seed=None):
        """Map a lat/lon patch of the GRS water map onto the cave extent.

        x_range, y_range: (min, max) of the cave in the map frame [m].
        x maps linearly to longitude and y to latitude.
        noise_scale multiplies the instrument's reported 1-sigma error.
        """
        self.x_range_ = x_range
        self.y_range_ = y_range
        self.lat_range_ = lat_range
        self.lon_range_ = lon_range
        self.noise_scale_ = noise_scale
        self.rng_ = np.random.default_rng(seed)

        # Columns: latitude, longitude, concentration, sigma, sigma with correction factors
        table = np.loadtxt(table_path)
        in_patch = ((table[:, 0] >= lat_range[0]) & (table[:, 0] <= lat_range[1]) &
                    (table[:, 1] >= lon_range[0]) & (table[:, 1] <= lon_range[1]))
        table = table[in_patch]
        if np.any(table[:, 2] >= NOT_APPLICABLE):
            raise ValueError('Selected lat/lon patch contains bins with no data')

        lats = np.unique(table[:, 0])
        lons = np.unique(table[:, 1])
        order = np.lexsort((table[:, 1], table[:, 0]))
        concentration = table[order, 2].reshape(len(lats), len(lons))
        sigma = table[order, 3].reshape(len(lats), len(lons))

        self.concentration_ = RegularGridInterpolator((lats, lons), concentration)
        self.sigma_ = RegularGridInterpolator((lats, lons), sigma)

    def to_lat_lon(self, x, y):
        """Convert a map-frame position to the latitude/longitude it represents"""
        u = (np.clip(x, *self.x_range_) - self.x_range_[0]) / (self.x_range_[1] - self.x_range_[0])
        v = (np.clip(y, *self.y_range_) - self.y_range_[0]) / (self.y_range_[1] - self.y_range_[0])
        lat = self.lat_range_[0] + v * (self.lat_range_[1] - self.lat_range_[0])
        lon = self.lon_range_[0] + u * (self.lon_range_[1] - self.lon_range_[0])
        return lat, lon

    def measure(self, x, y):
        """Noisy H2O concentration [weight %] at the robot's position"""
        lat, lon = self.to_lat_lon(x, y)
        sigma = self.sigma_((lat, lon)).item() * self.noise_scale_
        return self.concentration_((lat, lon)).item() + self.rng_.normal(0.0, sigma)

    def ground_truth(self, x, y):
        """Noise-free H2O concentration [weight %]. Evaluation only."""
        lat, lon = self.to_lat_lon(np.asarray(x), np.asarray(y))
        return self.concentration_(np.stack([lat, lon], axis=-1))
