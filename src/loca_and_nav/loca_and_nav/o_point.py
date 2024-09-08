import haversine as hs
from haversine import Unit

# original point of the global coordinate, this should be identical for all robots and should be close to the start point of the robot (within 500 m)
class O_Point():
    # Farm
    lat_0 = 30.537253708625634
    lon_0 = -96.42643216988164

    # TAMU CS
    # lat_0 = 30.6126599
    # lon_0 = -96.3431303

    # ART Lab
    # lat_0 = 30.617017659079263
    # lon_0 = -96.34105620732312
    
    # Unity simulation
    # lat_0 = 30.62277
    # lon_0 = -96.3346

    # Texas A & M AgriLife Research and Extension Center Corpus Christi
    # lat_0 = 27.782465 
    # lon_0 = -97.561410

    def __init__(self):
        self.lat_to_m = hs.haversine((self.lat_0, self.lon_0), (self.lat_0 + 0.001, self.lon_0), unit=Unit.METERS)*1000.0
        self.lon_to_m = hs.haversine((self.lat_0, self.lon_0), (self.lat_0, self.lon_0 + 0.001), unit=Unit.METERS)*1000.0
        self.m_to_lat = 1/self.lat_to_m
        self.m_to_lon = 1/self.lon_to_m
