"""Verify the live setter over real ROS parameter services; no costmap/motion."""
import sys
import threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'go2_3d'))
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from set_inflation import apply


rclpy.init()
global_map=Node('global_costmap',namespace='/global_costmap')
local_map=Node('local_costmap',namespace='/local_costmap')
for node in (global_map,local_map):
    node.declare_parameter('inflation_layer.inflation_radius',0.25)
    node.declare_parameter('resolution',0.05)
executor=MultiThreadedExecutor(num_threads=2)
executor.add_node(global_map); executor.add_node(local_map)
thread=threading.Thread(target=executor.spin,daemon=True); thread.start()
client=Node('test_go2_inflation_setter')
try:
    apply(client)
    assert all(node.get_parameter('inflation_layer.inflation_radius').value==0.05
               for node in (global_map,local_map))
    # Do not claim success when one server refuses the write.
    local_map.add_on_set_parameters_callback(
        lambda params:SetParametersResult(successful=False,reason='test refusal'))
    try:
        apply(client)
        raise AssertionError('Rejected parameter write was reported successful')
    except RuntimeError as error:
        assert 'change rejected' in str(error)
    print('PASS ROS live inflation: both actual parameters=0.05m; rejection is surfaced')
finally:
    client.destroy_node(); executor.shutdown(); thread.join(timeout=2)
    global_map.destroy_node(); local_map.destroy_node(); rclpy.shutdown()
