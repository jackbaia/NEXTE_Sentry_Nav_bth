# Takeoff and landing command
# Constants for takeoff_land_cmd
uint8 TAKEOFF = 1
uint8 LAND = 2

# Command field
uint8 takeoff_land_cmd

# Optional parameters
float64 takeoff_height  # Takeoff height in meters
float64 land_speed      # Landing speed in m/s 