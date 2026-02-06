from mountable import Mountable

class Rsu(Mountable):

    def __init__(self, actor, index, sensors_config, root_dir_path):
        save_dir_path = root_dir_path + f"/rsu{index}"
        super().__init__(index, sensors_config, actor, save_dir_path)

    def _attach_sensors(self):
        self._attach_sensor("camera")
        self._attach_sensor("depth_camera")
        self._attach_sensor("lidar")
        self._attach_sensor("radar")

    def save_data(self, vehicles, cavs):
        self._camera_saving()
        self._depth_camera_saving()
        self._lidar_saving()
        # self.__save_numeric_data_as_yaml(vehicles, cavs)




