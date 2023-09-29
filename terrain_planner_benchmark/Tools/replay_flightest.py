import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np

from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.legend_handler import HandlerLineCollection
import matplotlib.animation as animation

b = bagreader('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/_2023-07-17-13-05-12.bag')
# b = bagreader('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/_2023-07-20-06-12-33.bag')

print(b.topic_table)
TERRAIN_INFO = b.message_by_topic('/terrain_info')
info_df = pd.read_csv(TERRAIN_INFO)
print(info_df)

minimum_altitude = info_df["reference_minimum_altitude.data"].to_numpy()
maximum_altitude = info_df["reference_maximum_altitude.data"].to_numpy()
reference_position = info_df["reference_position.z"].to_numpy()
vehicle_position = info_df["vehicle_position.z"].to_numpy()
tracking_error_z = vehicle_position - reference_position
tracking_error_y = info_df["vehicle_position.y"].to_numpy() - info_df["reference_position.y"].to_numpy()
tracking_error_x = info_df["vehicle_position.x"].to_numpy() - info_df["reference_position.x"].to_numpy()

terrain_altitude = info_df["reference_terrain_altitude.data"].to_numpy()
distance_to_terrain = info_df["distance_to_terrain.data"].to_numpy()
time = info_df["header.stamp.secs"].to_numpy()
time_ns = info_df["header.stamp.nsecs"].to_numpy()/1e9
time = time + time_ns
segment_id = info_df["segment_id.data"].to_numpy()
time = time - time[0]

mask = segment_id > -1

minimum_altitude[~mask]=np.nan
maximum_altitude[~mask]=np.nan
reference_position[~mask]=np.nan
terrain_altitude[~mask]=np.nan
vehicle_position[~mask]=np.nan

fig = plt.figure('Reference', figsize=(4, 3))
ax = fig.add_subplot(1, 1, 1)
ax.set_ylabel('Altitude(AMSL) [m]')
ax.set_xlabel('Time [s]')
ymin, ymax = min(reference_position), max(reference_position)
handle_vehicle_position, = ax.plot([],[], lw=2, label='Vehicle')
reference, = ax.plot([],[], lw=2, label='Reference')
handle_terrain_altitude, = ax.plot([],[], lw=2, label='Terrain')
handle_min_altitude, = ax.plot([],[], lw=2)
handle_max_altitude, = ax.plot([],[], lw=2, color=handle_min_altitude.get_color())
handle_marker = ax.scatter([], [], s=20, facecolor='red')

index_increment = np.arange(0,len(reference_position), 1)
capped_index_increment = np.arange(0,len(reference_position), 1)
capped_index_increment = np.roll(capped_index_increment, 499)
capped_index_increment[0:499] = np.zeros_like(capped_index_increment[0:499])


idx = [(s,e) for s,e in zip(capped_index_increment, index_increment)]

def init():
    reference.set_data([], []) 
    handle_min_altitude.set_data([], []) 
    handle_max_altitude.set_data([], [])
    handle_vehicle_position.set_data([], [])
    handle_terrain_altitude.set_data([], [])

    return reference, handle_min_altitude, handle_max_altitude, handle_vehicle_position, handle_terrain_altitude


def animate(i):
  id = idx[i]
  ax.collections.clear()
  reference.set_data(time[id[0]: id[1]], reference_position[id[0]:id[1]])
  # handle_min_altitude.set_data(time[id[0]: id[1]], minimum_altitude[id[0]:id[1]])
  reference_color = handle_min_altitude.get_color()
  constraint_handle = ax.fill_between(time[id[0]: id[1]],  minimum_altitude[id[0]:id[1]], maximum_altitude[id[0]:id[1]], alpha=0.4, label='_nolegend_', color=reference_color)
  # handle_max_altitude.set_data(time[id[0]: id[1]], maximum_altitude[id[0]:id[1]])
  handle_vehicle_position.set_data(time[id[0]: id[1]], vehicle_position[id[0]:id[1]])
  handle_terrain_altitude.set_data(time[id[0]: id[1]], terrain_altitude[id[0]:id[1]])
  # TODO: handle data with constraint-> handle_constraint
  ax.scatter(time[id[1]], vehicle_position[id[1]], c=handle_vehicle_position.get_color())
#   ax.scatter(x, y, s=20, c='red')
  ax.set_xlim(time[id[0]], time[id[0]]+70)
  ax.set_ylim(ymin-200, ymax+80)
  ax.set_ylabel('Altitude [m]')
  ax.set_xlabel('Time [s]')
  ax.legend([handle_vehicle_position, reference, handle_terrain_altitude, constraint_handle], ["Vehicle", "Reference", "Terrain", "Min/Max Alt"]\
      , framealpha=1, loc='lower center', ncol=2)
  ax.grid(True)
  return reference, handle_min_altitude, handle_max_altitude, handle_vehicle_position,

anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(reference_position)-500, interval=30, blit=False)
fig.tight_layout()

anim.save(
    "circle_anim.mov",
    codec="png",
    dpi=100,
    bitrate=-1,
    savefig_kwargs={"transparent": True, "facecolor": "none"},
)

plt.show()
