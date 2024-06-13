# thruster_config.py
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from ngc_utils.thruster_models import AzimuthThruster, PropellerRudderUnit, TunnelThruster

def load_thrusters_from_yaml(logger, yaml_package_name, propulsion_config_file, config_params):
    yaml_package_path = get_package_share_directory(yaml_package_name)
    propulsion_config_path = os.path.join(yaml_package_path, propulsion_config_file)

    with open(propulsion_config_path, 'r') as file:
        config = yaml.safe_load(file)

    thrusters = []
    for thruster_name, thruster_config in config.items():
        thruster_type = thruster_config['type']
        
        if thruster_type == 'propeller_with_rudder':
            logger.info(f"Found propeller with rudder: {thruster_name}")
            thrusters.append(PropellerRudderUnit(thruster_config, config_params))
        elif thruster_type == 'tunnel_thruster':
            logger.info(f"Found tunnel thruster: {thruster_name}")
            thrusters.append(TunnelThruster(thruster_config, config_params))
        elif thruster_type == 'azimuth_thruster':
            logger.info(f"Found azimuth thruster: {thruster_name}")
            thrusters.append(AzimuthThruster(thruster_config, config_params))

    return thrusters
