# The filtered global position (e.g. fused GPS and accelerometers). 
# The position is in GPS-frame (right-handed, Z-up). 
# It is designed as scaled integer message since the resolution of float is not sufficient.
# Latitude, expressed as * 1E7; Longitude, expressed as * 1E7
# Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
# Altitude above ground in meters, expressed as * 1000 (millimeters)
# Ground X, Y, Z Speed (Latitude), expressed as m/s * 100
# Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX

uint8 ip
uint32 time_boot_ms
int32 lat 
int32 lon
int32 alt

float32 relative_alt
float32 vx
float32 vy
float32 vz
float32 hdg

