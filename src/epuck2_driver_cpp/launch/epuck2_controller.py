from launch import LaunchDescription
from launch_ros.actions import Node
# id2ip = {
#     4857: "192.168.28.66",
#     4847: "192.168.28.146",
#     4791: "192.168.28.250",
#     5082: "192.168.28.42",
# }
id2ip = {
    4857: "192.168.12.151",
    4847: "192.168.12.6",
    4791: "192.168.12.44",
    5082: "192.168.12.127",
}
id_list = []
ip_list = []
for (key, value) in id2ip.items():
    ip_list.append(value)
    id_list.append(key)
# init pose x y z, enable cam ,enable ground
init_list = [[0.0, 0.0, 0.0, False, False]]*ip_list.__len__()

if ip_list.__len__() != id_list.__len__() or ip_list.__len__() != init_list.__len__():
    exit(1)
is_single_robot = 1 if ip_list.__len__() == 1 else 0


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace="epuck2_robot_"+str(id_list[i]),
            package="epuck2_driver_cpp",
            executable="epuck2_driver_node",
            name="epuck2_driver_node_"+str(id_list[i]),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"epuck2_id": id_list[i]},
                {"epuck2_address": ip_list[i]},
                {"epuck2_name": "epuck2_robot_"+str(id_list[i])},
                {"xpos": init_list[i][0]},
                {"ypos": init_list[i][1]},
                {"theta": init_list[i][2]},
                {"cam_en": init_list[i][3]},
                {"floor_en": init_list[i][4]}
            ]
        ) for i in range(ip_list.__len__())
    ])
