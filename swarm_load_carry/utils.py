from geometry_msgs.msg import TransformStamped

# Find Euler distance between two points in 3D space
# Inputs: points p1, p2 as 3-tuples
# Outputs: dist - distance between points (in input units)
def dist_euler_3D(p1, p2):
    dist = ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)**0.5

    return dist

# Check if a point is within an error radius of another point
# Inputs: points p1, p2 as 3-tuples
# Outputs: true if within radius, false otherwise
def within_radius_3D(p1, p2, rad):
    if dist_euler_3D(p1,p2) < rad:
        return True
    else:
        return False
    

# Build and send a tf
# Inputs: time, names of parent and child frames, position (as np array x,y,z), attitude (as quaternionic array), broadcaster (static or dynamic broadcaster to send tf over)
# Outputs: None 
def broadcast_tf(time, frame_parent, frame_child, pos, att, broadcaster):
    # Broadcast frame transform
    t = TransformStamped()

    t.header.stamp = time
    t.header.frame_id = frame_parent
    t.child_frame_id = frame_child

    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]

    t.transform.rotation.x = float(att.x)
    t.transform.rotation.y = float(att.y)
    t.transform.rotation.z = float(att.z)
    t.transform.rotation.w = float(att.w)

    broadcaster.sendTransform(t)


