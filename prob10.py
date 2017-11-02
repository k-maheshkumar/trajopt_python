
request = { 'duration': 15,
            'samples' : 5,
            'duration' : 10,
            'maxIteration': 100,
            'joints': [
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0,  'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0,  'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96}
           ]}
request1 = { 'duration': 15,
            'samples' : 5,
            'duration' : 10,
            'maxIteration': 100,
            'joints': [
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0,  'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0,  'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96},
                {'max_velocity': 10.0, 'upper_joint_limit': 2.96, 'min_velocity': -10.0,
                                'id': 0, 'lower_joint_limit': -2.96}
           ]}
startState = [-0.49197958189616936, 1.4223062659337982, -1.5688299779644697, -1.3135004031364736, 1.5696229411153653, 1.5749627479696093, 1.5708037563007493]
endState = [-2.0417782994426674, 0.9444594031189716, -1.591006403858707, -1.9222844444479184, 1.572303282659756, 1.5741716208788483, 1.5716145442929421]


for i in range(7):
    # request["joints"][i].update(
    #     {"start": startState[i], "end": endState[i]})
    # request1["joints"][i].update(
    #     {"start": endState[i], "end": startState[i]})
    pose = {}
    pose["start"] = startState[i]
    pose["end"] = endState[i]
    pose1 = {}
    pose1["end"] = startState[i]
    pose1["start"] = endState[i]
    request["joints"][i].update(pose)
    # print request
    request1["joints"][i].update(pose1)
    # print request1

    # print "pose:", pose
    # print "pose1:", pose1
    # request["joints"].append(joint)
# print startState[0]
print "request:", request
print "request1:", request1


def updateJoints(startState = [0], endState = [0]):
    for i in range(numJoints):
        joint = {}
        joint["start"] = startState[i]
        joint["end"] = endState[i]
        request["joints"][i].update(joint)
        joint1 = {}
        joint1["end"] = startState[i]
        joint1["start"] = endState[i]
        request1["joints"][i].update(joint1)