# Utilities

QIB_TO_RUNS_JOINT_ORDER = [0, 1, 13, 14, 15, 16, 17,  2,  3,  4,  5,  6,  7, 8, 9, 10, 11, 12, 19, 20, 21, 22, 23, 18, 24]
RUNS_TO_QIB_JOINT_ORDER = [0, 1,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 2, 3,  4,  5,  6, 23, 18, 19, 20, 21, 22, 24]
JOINT_NAMES = ["HY", "HP", "LSP", "LSR", "LEY", "LER", "LWY", "LHYP", "LHR", "LHP", "LKP", "LAP", "LAR", "RHR", "RHP", "RKP", "RAP","RAR","RSP","RSR","REY","RER","RWY","LH","RH"]

WEBOTS_TO_RUNSWIFT = [-1, -1, -1, -1, -1, -1, -1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, -1, -1, -1, -1, -1, -1, -1]
#RUNSWIFT_TO_WEBOTS = [7, 8, 9, 10, 11, 12, -1, 13, 14, 15, 16, 17, 18]


# TODO
# This method does not work as intended.
# Some edge case is causing frequent infinite loops.
# The issue is that for some joints, the start and end vals are the
# same, leading to a step size of zero
def Lerp(start_val, end_val, time_step):
    step = round((end_val-start_val)/time_step, 5)
    curr_val = start_val
    lerp_vals = [] # does not include start
    if step > 0:
        while curr_val+step < end_val:
            print(step, curr_val+step-end_val)
            print("L0.6a")
            curr_val += step
            print("L0.7a")
            lerp_vals.append(curr_val)
    elif step < 0:
        while curr_val+step > end_val:
            print(step, curr_val+step-end_val)
            print("L0.6b")
            curr_val += step
            print("L0.7b")
            lerp_vals.append(curr_val)
    else:
        # raise Exception(f"Step size is {step} which is close to zero")
        pass
    return lerp_vals

# Takes 2 lists of joint vals and returns their corresponding errors
# as a list
def JointErrors(curr_vals, target_vals):
    if len(curr_vals) == len(target_vals):
        errors = [abs(curr_val - target_val)
                    for curr_val, target_val in zip(curr_vals, target_vals)]
        return errors
    else:
        raise Exception("Current and Target joint value lists don't have the same length.")


# order: 1 = qibullet->runswift, 2 = runswift->qibullet
def ChangeNaoJointOrder(jointvals, order=1):
    reordered_joints = []
    if order == 1:
        for i in QIB_TO_RUNS_JOINT_ORDER:
            reordered_joints.append(jointvals[i])
    elif order == 2:
        for i in RUNS_TO_QIB_JOINT_ORDER:
            reordered_joints.append(jointvals[i])
    elif order == 3:
        for i in WEBOTS_TO_RUNSWIFT:
            if i <= 0:
                reordered_joints.append(0) # TODO: Should be whatever previous position, not zero
            else:
                reordered_joints.append(jointvals[i-2])

    return reordered_joints

if __name__ == "__main__":
    # print(Lerp(0.0, 3.14, 5.0))
    pass