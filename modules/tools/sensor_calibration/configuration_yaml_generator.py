import os
import yaml

class ConfigYaml(object):
    """generate yaml configuration for next-step calibration service.
    automatically generate calibration configuration yaml file. help user to
    input the initial extrinsics values or path(lidar, camera), intrinsics path(camera),
    sensor name if needs changes.
    """
    def __init__(self, supported_calibrations=['lidar_to_gnss', 'camera_to_lidar']):
        self._task_name = 'unknown'
        self._supported_tasks = supported_calibrations
        print('calibration service now support: {}'.format(self._supported_tasks))

    def load_sample_yaml_file(self, task_name, sample_file=None):
        if sample_file is None:
            sample_file = os.path.join(os.path.dirname(__file__),
                                'config', task_name + '_sample_config.yaml')
        try:
            with open(sample_file, 'r') as f:
                data = yaml.safe_load(f)
        except IOError:
            raise ValueError('cannot open the sample configure yaml file at {}'.format(sample_file))
        return data

    def _generate_lidar_to_gnss_calibration_yaml(self, in_data, lidar_folder_name, gnss_folder_name):
        in_data['sensor_files_directory'] = os.path.join('.', lidar_folder_name, "")
        in_data['odometry_file'] = os.path.join('.', gnss_folder_name,'odometry')
        return in_data

    def _generate_camera_to_lidar_calibration_yaml(self):
        raise ValueError("not implemented yet: camera_to_lidar calibration yaml file generator")

    def get_task_name(self):
        if self._task_name == 'unknown':
            raise ValueError('have not set the task name, the valid task names'
                            'are: {}'.format(self._supported_tasks))
        return self._task_name

    def generate_task_config_yaml(self, task_name, source_sensor, dest_sensor,
                                source_folder, dest_folder, out_config_file,
                                in_config_file=None):
        self._task_name = task_name
        out_data = self.load_sample_yaml_file(task_name=task_name, sample_file=in_config_file)
        out_data['source_sensor'] = source_sensor
        out_data['destination_sensor'] = dest_sensor
        if not task_name in self._supported_tasks:
            raise ValueError('does not support the calibration task: {}'.format(
                            task_name))

        if self._task_name == 'lidar_to_gnss':
            out_data = self._generate_lidar_to_gnss_calibration_yaml(
                        out_data, source_folder, dest_folder)
        elif self._task_name == 'camera_to_lidar':
            out_data = self._generate_camera_to_lidar_calibration_yaml()

        print(out_data)
        try:
            with  open(out_config_file, 'w') as f:
                yaml.safe_dump(out_data, f)
        except:
            raise ValueError('cannot generate the task config yaml file at {}'.format(out_config_file))
        return True