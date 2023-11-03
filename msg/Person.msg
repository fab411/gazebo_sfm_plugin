int32               id            # Object ID
string              name          # Object Type
geometry_msgs/Point position      # 3D coordinates info of the center position of the object
geometry_msgs/Point velocity      # the velocity of the object moving. Unit: m/s
float64             reliability   # The detection probability. 
string[]            tagnames
string[]            tags



##Point_msg
#float64 x
#float64 y
#float64 z