int8 id                       # id from detector (presumes < 255)
string label                  # assigned label
uint8[] rgb_color             # detected color (red, green, blue)
geometry_msgs/Point center    # center location in image pixels (upper left)
float32 radius                # radius in image pixels
geometry_msgs/Point position  # 3D projected position in target frame (meters)
