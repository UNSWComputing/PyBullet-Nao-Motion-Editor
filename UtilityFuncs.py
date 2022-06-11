# Utilities

def Lerp(start, end, duration):
    step = (end-start)/duration
    curr_val = start
    lerp_vals = [] # does not include start
    while curr_val+step < end:
        curr_val += step
        lerp_vals.append(curr_val)
    return lerp_vals

if __name__ == "__main__":
    # print(Lerp(0.0, 3.14, 5.0))
    pass