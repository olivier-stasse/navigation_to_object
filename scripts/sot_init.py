from dynamic_graph import plug
from dynamic_graph.ros import *
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize (robot)
from dynamic_graph.sot.pattern_generator.walking import CreateEverythingForPG , walkAndrei

CreateEverythingForPG(robot, solver)

ros = Ros(robot)
ros.rosExport.add('vector3Stamped', 'vref', '/pattern_generator/velocity')
