import os
from typing import MutableMapping

import yaml

from .structures import Track
from matplotlib import pyplot as plt

__all__ = ["tracks"]

path = os.path.join(os.getcwd(), 'src/gokart-core/packages/motion/control/pure_pursuit_controller/param')
tracks_file = os.path.join(path, "tracks.yaml")
with open(tracks_file) as f:
    track_yaml = yaml.load(f, Loader=yaml.FullLoader)

for track_name in track_yaml:
    track_yaml[track_name]["background"] = plt.imread(os.path.join(path, "rieter.png"))
    track_yaml[track_name]["scale_factor"] = 1 / 7.5
    track_yaml[track_name]["name"] = track_name

tracks: MutableMapping[str, Track] = {}
for track_name in track_yaml:
    tracks[track_name] = Track.from_dict(track_yaml[track_name])
