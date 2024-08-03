from geometry_msgs.msg import Point, Pose, Quaternion


TAGS = {
    1: "blue_hexagon",
    2: "green_triangle",
    3: "red_cube",
    4: "gold_obs_0",
    5: "gold_obs_1",
    6: "gold_obs_2",
    7: "gold_obs_3",
}
OBJECTS_POSES = {
    1: {
        "table": Pose(Point(8, -2.14, 0), Quaternion(0, 0, -0.98, 1)),
        "middle_point": Pose(Point(8.7, -1.5, 0), Quaternion(0, 0, -1, 1)),
        "cylinder": Pose(Point(12.6, -0.9, 0), Quaternion(0, 0, 1, 1)),
    },
    2: {
        "table": Pose(Point(7.7, -3.95, 0), Quaternion(0, 0, 1.02, 1)),
        "middle_point": Pose(Point(8.7, -4.2, 0), Quaternion(0, 0, 1, 1)),
        "cylinder": Pose(Point(11.6, -0.9, 0), Quaternion(0, 0, 1, 1)),
    },
    3: {
        "table": Pose(Point(7.5, -2.14, 0), Quaternion(0, 0, -0.98, 1)),
        "middle_point": Pose(Point(8.7, -1.5, 0), Quaternion(0, 0, -1, 1)),
        "cylinder": Pose(Point(10.6, -0.9, 0), Quaternion(0, 0, 1, 1)),
    },
}

# BLUE_OBJECT_POSITION = Pose(Point(8, -2.2, 0), Quaternion(0, 0, -1, 1))  # ID1
# GREEN_OBJECT_POSITION = Pose(Point(7.7, -3.95, 0), Quaternion(0, 0, 1, 1))  # ID2
# RED_OBJECT_POSITION = Pose(Point(7.5, -2.2, 0), Quaternion(0, 0, -1, 1))  # ID3


# BLUE_DESTINATION_POSITION = Pose(Point(12.6, -0.9, 0), Quaternion(0, 0, 1, 1))  # ID1
# GREEN_DESTINATION_POSITION = Pose(Point(11.6, -0.9, 0), Quaternion(0, 0, 1, 1))  # ID2
# RED_DESTINATION_POSITION = Pose(Point(10.6, -0.9, 0), Quaternion(0, 0, 1, 1))  # ID3

# BLUE_SAFE_POSITION = Pose(Point(8.7, -1.5, 0), Quaternion(0, 0, -1, 1))
# GREEN_SAFE_POSITION = Pose(Point(8.7, -4.2, 0), Quaternion(0, 0, 1, 1))
# RED_SAFE_POSITION = Pose(Point(8.7, -1.5, 0), Quaternion(0, 0, -1, 1))

MIDDLE_SAFE_POSITION = Pose(Point(9.5, -4.2, 0), Quaternion(0, 0, 1, 1))
START_SAFE_POINT = Pose(Point(8.7, -1, 0), Quaternion(0, 0, -1, 1))
