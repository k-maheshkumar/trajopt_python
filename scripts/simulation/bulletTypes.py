from collections import namedtuple

JointInfo = namedtuple('JointInfo', ['joint_index', 'joint_name', 'joint_type', 'q_index', 'u_index', 'flags',
                                     'joint_damping', 'joint_friction', 'joint_lower_limit', 'joint_upper_limit',
                                     'joint_max_force', 'joint_max_velocity', 'link_name', 'joint_axis',
                                     'parent_frame_pos', 'parent_frame_orn', 'parent_index'])

CastClosestPointInfo = namedtuple('ContactPointInfo', ['contact_flag', 'body_unique_id_a', 'body_unique_id_b', 'link_index_a',
                                         'link_index_b', 'position_on_a', 'position_on_a1', 'position_on_b', 'contact_normal_on_b',
                                         'contact_distance', 'contact_fraction', 'normal_force'])

ClosestPointInfo = namedtuple('ClosestPointInfo', ['contact_flag', 'body_unique_id_a', 'body_unique_id_b', 'link_index_a',
                                         'link_index_b', 'position_on_a', 'position_on_b', 'contact_normal_on_b',
                                         'contact_distance', 'normal_force'])

