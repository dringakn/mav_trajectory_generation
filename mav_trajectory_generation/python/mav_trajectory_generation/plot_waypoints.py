#!/usr/bin/env python3
"""
plot_waypoints.py

Usage:
    # Static map with OSM background
    python plot_waypoints.py --mode static --output waypoints_static.png

    # Interactive Folium map
    python plot_waypoints.py --mode folium --output waypoints.html
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt
import geopandas as gpd
import contextily as ctx
import folium
from shapely.geometry import Point
from geographic_converter import GeodeticConverter

def load_waypoints():
    """
    Return a DataFrame with columns ['lat','lon','alt'].
    Replace this with pd.read_csv(...) if you have an external file.
    """
    data = [
        (49.7939, 9.9512, 0.0),  # Wurzburg center (origin)
        (49.7945, 9.9520, 123.5),
        (49.7950, 9.9505, 118.7),
        (49.7930, 9.9490, 122.2),
    ]
    return pd.DataFrame(data, columns=['lat','lon','alt'])

def convert_roundtrip(df):
    """
    Adds columns north,east,down and lat_back,lon_back,alt_back to df.
    """
    gc = GeodeticConverter()
    # use first row as reference origin
    lat0, lon0, alt0 = df.loc[0, ['lat','lon','alt']]
    gc.initialise_reference(lat0, lon0, alt0)

    # Geodetic → NED
    df[['north','east','down']] = df.apply(
        lambda r: pd.Series(gc.geodetic2ned(r.lat, r.lon, r.alt)),
        axis=1
    )
    # NED → Geodetic
    df[['lat_back','lon_back','alt_back']] = df.apply(
        lambda r: pd.Series(gc.ned2geodetic(r.north, r.east, r.down)),
        axis=1
    )
    return df

def plot_static(df, output):
    # Create GeoDataFrame in WGS84
    geom = [Point(xy) for xy in zip(df['lon_back'], df['lat_back'])]
    gdf = gpd.GeoDataFrame(df, geometry=geom, crs="EPSG:4326")
    # Project to Web Mercator
    gdf = gdf.to_crs(epsg=3857)

    # Plot
    fig, ax = plt.subplots(figsize=(8, 6))
    gdf.plot(ax=ax, marker='o', color='red', markersize=80, label='Waypoints')
    ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)
    ax.set_axis_off()
    ax.legend()
    plt.tight_layout()
    fig.savefig(output, dpi=300)
    print(f"Static map saved to {output}")

def plot_folium(df, output):
    # Center map on mean lat/lon
    center = [df['lat_back'].mean(), df['lon_back'].mean()]
    m = folium.Map(location=center, zoom_start=15, tiles='OpenStreetMap')
    for _, row in df.iterrows():
        folium.Marker(
            [row.lat_back, row.lon_back],
            popup=f"Alt: {row.alt_back:.1f} m"
        ).add_to(m)
    m.save(output)
    print(f"Interactive map saved to {output}")

def main():
    p = argparse.ArgumentParser(description="Plot GPS waypoints in Folium or static OSM map")
    p.add_argument('--mode', choices=['folium','static'], default='static',
                   help='“folium” for interactive map, “static” for Matplotlib+Contextily')
    p.add_argument('--output', help='Output filename (for folium: .html; for static: .png)')
    args = p.parse_args()

    df = load_waypoints()
    df = convert_roundtrip(df)

    if args.mode == 'static':
        out = args.output or 'waypoints_static.png'
        plot_static(df, out)
    else:
        out = args.output or 'waypoints.html'
        plot_folium(df, out)
        
    print("Waypoints:")
    # print(df[['lat_back', 'lon_back', 'alt_back']].to_string(index=False))
    print(df.to_string(index=False))

if __name__ == '__main__':
    main()
