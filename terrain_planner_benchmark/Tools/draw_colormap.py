import pylab as pl
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys

def main():
    path=sys.argv[1]
    data_df = pd.read_csv(path)

    altitude = data_df["z"]

    min_altitude = np.min(altitude)
    max_altitude = np.max(altitude)

    a = np.array([[min_altitude,max_altitude]])/1000
    pl.figure(figsize=(1, 3))
    img = pl.imshow(a, cmap="plasma")
    pl.gca().set_visible(False)

    # -  4-tuple of floats *rect* = ``[left, bottom, width, height]``.
    cax = pl.axes([0.1, 0.1, 0.2, 0.8])
    cbar = pl.colorbar(orientation="vertical", cax=cax)
    cbar.set_label('Altitude [km]')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
