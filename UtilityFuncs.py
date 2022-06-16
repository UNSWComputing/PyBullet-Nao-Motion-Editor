# Utilities

QIB_TO_RUNS_JOINT_ORDER = [0, 1, 13, 14, 15, 16, 17, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 19, 20, 21, 22, 23, 18, 24]
# RUNS_TO_QIB_JOINT_ORDER = []

def Lerp(start, end, duration):
    step = (end-start)/duration
    curr_val = start
    lerp_vals = [] # does not include start
    while curr_val+step < end:
        curr_val += step
        lerp_vals.append(curr_val)
    return lerp_vals

# order: 1 = qibullet->runswift, 2 = runswift->qibullet
def ChangeNaoJointOrder(jointvals, order=1):
    reordered_joints = []
    if order == 1:
        for i in QIB_TO_RUNS_JOINT_ORDER:
            reordered_joints.append(jointvals[i])
    elif order == 2:
        # TODO
        pass

    return reordered_joints

if __name__ == "__main__":
    # print(Lerp(0.0, 3.14, 5.0))
    pass