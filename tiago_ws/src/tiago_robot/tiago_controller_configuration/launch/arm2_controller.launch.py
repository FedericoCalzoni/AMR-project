import os

from ament_index_python.packages import get_package_share_directory
from controller_manager.launch_utils import generate_load_controller_launch_description


def generate_launch_description():
    return generate_load_controller_launch_description(
        controller_name='arm_velocitiy_controller',
        controller_type='velocity_controllers/JointGroupVelocityController',
        controller_params_file=os.path.join(
            get_package_share_directory('tiago_controller_configuration'),
            'config', 'arm_controller_2.yaml'))
