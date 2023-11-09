# members in the group and position
people_msgs/Person[]    group_members
geometry_msgs/Pose      midpoint

# allowed types are 'circle', 'line', or 'other'
string                  type

# for type 'line' this represents the length. for type 'circle' this is the radius
float64                 size