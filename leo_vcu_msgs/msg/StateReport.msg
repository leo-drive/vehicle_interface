std_msgs/Header header
# ID: 1042
uint8 fuel                              # Fuel Level of Vehicle
uint8 blinker                           # Blinker Status
bool headlight                          # Headlight Status
uint8 wiper                             # Wiper Status
uint8 gear                              # Gear Status 1:PARK 2:REVERSE 3:NEUTRAL 4:DRIVE
uint8 mode                              # Mode Status 1:AUTO 2:REMOTE
uint8 hand_brake                        # Handbrake Status
uint8 horn                              # Horn Status
# ID: 1043
uint8 steering_intervention             # Steering Intervention Status
uint8 brake_intervention                # Brake Intervention Status
uint8 acc_pedal_intervention            # Acceleration Pedal Intervention Status
uint8 ready                             # Ready Information
uint8 motion_allow                      # Motion Allow Information
uint8 throttle                          # Throttle Level of Vehicle
uint8 brake                             # Brake Level of Vehicle
uint16 front_steer                      # Front Steer Angle (Deg)
# ID: 1044
uint8 motor_temp                        # Motor Temperature of Vehicle
uint16 motor_rpm                        # Motor RPM of Vehicle
uint8 kl75                              # KL75 (Ignition) status of Vehicle
