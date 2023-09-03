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

b = bagreader('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/_2023-09-02-08-43-58.bag')
# b = bagreader('/home/jaeyoung/catkin_ws/src/terrain-navigation/output/_2023-07-20-06-12-33.bag')

def colorLineSegments(time, reference_position, segment_id):
    points = np.array([time, reference_position]).T.reshape(-1,1,2)
    segments = np.concatenate([points[:-2],points[1:-1], points[2:]], axis=1)
    intensity = (segment_id - 2) / 8
    colors = [cmap(k) for k in intensity[:-1]]
    line_segments = LineCollection(segments, cmap=cmap, colors=colors, linewidth=2)
    return line_segments


class HandlerColorLineCollection(HandlerLineCollection):
    def create_artists(self, legend, artist ,xdescent, ydescent,
                        width, height, fontsize,trans):
        x = np.linspace(0,width,self.get_numpoints(legend)+1)
        y = np.zeros(self.get_numpoints(legend)+1)+height/2.-ydescent
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        lc = LineCollection(segments, cmap=artist.cmap,
                     transform=trans)
        lc.set_array(x)
        lc.set_linewidth(artist.get_linewidth())
        return [lc]

print(b.topic_table)
TERRAIN_INFO = b.message_by_topic('/terrain_info')
info_df = pd.read_csv(TERRAIN_INFO)
print(info_df)

minimum_altitude = info_df["reference_minimum_altitude.data"].to_numpy()
maximum_altitude = info_df["reference_maximum_altitude.data"].to_numpy()
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
# mask = np.logical_and(mask, segment_id!=2)
# mask = np.logical_and(mask, segment_id!=4)
# mask = np.logical_and(mask, segment_id!=6)
# mask = np.logical_and(mask, segment_id!=8)


minimum_altitude[~mask]=np.nan
maximum_altitude[~mask]=np.nan
reference_position[~mask]=np.nan
terrain_altitude[~mask]=np.nan
vehicle_position[~mask]=np.nan

fig = plt.figure('Reference', figsize=(7, 3))
ax = fig.add_subplot(1, 1, 1)

constraint_handle = ax.fill_between(time, minimum_altitude, maximum_altitude, alpha=0.4)
terrain_handle, = ax.plot(time, terrain_altitude, label='Terrain')
def colorlist2(c1, c2, num):
    l = np.linspace(0,1,num)
    a = np.abs(np.array(c1)-np.array(c2))
    m = np.min([c1,c2], axis=0)
    s  = np.sign(np.array(c2)-np.array(c1)).aswhichtype(int)
    s[s==0] =1
    r = np.sqrt(np.c_[(l*a[0]+m[0])[::s[0]],(l*a[1]+m[1])[::s[1]],(l*a[2]+m[2])[::s[2]]])
    return r

cmap = cm.get_cmap('gist_rainbow')

line_segments = colorLineSegments(time, reference_position, segment_id)
ax.add_collection(line_segments)

ax.set_xlim([0.0, 750.0])
ax.grid(True)
ax.set_ylabel('Altitude [m]')
ax.set_xlabel('Time [s]')
# ax.legend()
ax.legend([line_segments, terrain_handle, constraint_handle], ["Reference Altitude", "Terrain Altitude", "Altitude Limits"],\
    handler_map={line_segments: HandlerColorLineCollection(numpoints=8), }, framealpha=1, loc='upper center')

fig.tight_layout()

fig2 = plt.figure('Distance', figsize=(7, 3))
ax2 = fig2.add_subplot(1, 1, 1)
# handler_dist_to_terrain, = ax2.plot(time, distance_to_terrain, label='Distance to terrain')
line_segments2 = colorLineSegments(time, distance_to_terrain, segment_id)
ax2.add_collection(line_segments2)
handler_minmax = ax2.hlines(y=50, xmin=0.0, xmax=750.0, color='r', linestyles='--', label='Distance to terrain')
handler_minmax2 = ax2.hlines(y=120, xmin=0.0, xmax=750.0, color='r', linestyles='--', label='Distance to terrain')
ax2.set_xlim([0.0, 750.0])
ax2.grid(True)
ax2.set_ylabel('Distance [m]')
ax2.set_xlabel('Time [s]')

ax2.legend([line_segments2, handler_minmax], ["Terrain Distance", "Min/Max Distance"],\
    handler_map={line_segments2: HandlerColorLineCollection(numpoints=8), }, framealpha=1, loc='lower center')

fig2.tight_layout()

fig3 = plt.figure('Tracking Error', figsize=(7, 3))
ax3 = fig3.add_subplot(1, 1, 1)

last_time=0.0
for id in np.arange(1, np.max(segment_id)+1):
    mask = segment_id == id
    print(segment_id)
    span_color = cmap((id - 2) / 8)
    print("id: ", id)
    print("color: ", span_color)
    print("min time: ", np.min(time[mask]))
    print("max time: ", np.max(time[mask]))
    if id < 10:
        max_time = np.max(time[mask])
    else:
        max_time = 750.0
    ax3.axvspan(last_time, max_time, -10, 10, facecolor=span_color, alpha=0.2)
    last_time = np.max(time[mask])

ax3.plot(time, tracking_error_x, label='x')
ax3.plot(time, tracking_error_y, label='y')
ax3.plot(time, tracking_error_z, label='z')
ax3.hlines(y=0, xmin=0.0, xmax=750.0, color='r', linestyles='--', label='Reference')
ax3.set_xlim([0.0, 750.0])
ax3.grid(True)
ax3.set_ylabel('Tracking Error [m]')
ax3.set_xlabel('Time [s]')
ax3.legend()

horizontal_track_error = np.sqrt(np.square(tracking_error_x) + np.square(tracking_error_y))
mean_horizontal_track_error = np.mean(horizontal_track_error)
std_horizontal_track_error = np.std(horizontal_track_error)
mean_vertical_track_error = np.mean(np.abs(tracking_error_z))
std_vertical_track_error = np.std(np.abs(tracking_error_z))
print("horizontal track_error : ", mean_horizontal_track_error, '+-', std_horizontal_track_error)
print("  -  max   : ", np.max(np.abs(horizontal_track_error)))
print("  - index: ", time[np.argmax(np.abs(horizontal_track_error))])
print("vertical track error   : ", mean_vertical_track_error, '+-', std_vertical_track_error)
print("  -  max   : ", np.max(np.abs(tracking_error_z)))
print("  - index: ", time[np.argmax(tracking_error_z)])

fig3.tight_layout()

plt.show()
