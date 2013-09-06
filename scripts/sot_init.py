# Import ROS
from dynamic_graph.ros import *

#FIXME: initialize the pattern-generator

# Create ros bindings
ros = Ros(robot)

# Subscribe to reference velocity
ros.rosImport.add('vector3Stamped', 'vref', '/pattern-generator/velocity')

# Plug into the pattern generator.
plug(ros.rosImport.vref, pg.vref)

# Start the pattern-generator.
pg.start()
