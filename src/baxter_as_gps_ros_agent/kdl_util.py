import PyKDL

def array_to_kdl_JntArray(array):
    kdl_array = PyKDL.JntArray(7)
    for i in range(7):
        kdl_array[i] = array[i]
    return kdl_array 
