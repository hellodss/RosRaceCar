#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_model():
    # 初始化ROS节点
    rospy.init_node('spawn_model_node')

    # 等待服务可用
    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        # 创建服务代理
        spawn_model_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        # 读取SDF模型
        with open("src/PutModel/model/construction_cone_red/model.sdf", "r") as f:
            model_xml = f.read()

        # 设置模型的位置
        model_pose = Pose()
        model_pose.position.x = 1.0  # 设置X坐标
        model_pose.position.y = 0.0  # 设置Y坐标
        model_pose.position.z = 0.0  # 设置Z坐标
        model_pose.orientation.w = 1.0  # 默认方向

        # 调用服务生成模型
        spawn_model_service("my_model", model_xml, "", model_pose, "world")
        rospy.loginfo("Model spawned successfully.")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


if __name__ == '__main__':
    spawn_model()
