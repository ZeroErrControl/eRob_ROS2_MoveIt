from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("eRobo3", package_name="twojoints_test").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
