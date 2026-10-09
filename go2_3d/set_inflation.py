#!/usr/bin/env python3
"""Set and read back both live costmap inflation radii; no motion writes."""
import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, GetParameters


def result(node, future):
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    if not future.done():
        raise RuntimeError('Costmap parameter request timed out')
    return future.result()


def apply(node):
    radius = 0.05
    for name in ('/global_costmap/global_costmap', '/local_costmap/local_costmap'):
        setter = node.create_client(SetParameters, name+'/set_parameters')
        getter = node.create_client(GetParameters, name+'/get_parameters')
        if not setter.wait_for_service(timeout_sec=5.0) or not getter.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'{name}: parameter services unavailable; keep navigation running')
        request = SetParameters.Request()
        request.parameters = [Parameter('inflation_layer.inflation_radius', value=radius).to_parameter_msg()]
        reply = result(node, setter.call_async(request))
        if len(reply.results) != 1 or not reply.results[0].successful:
            raise RuntimeError(f'{name}: radius change rejected: {reply.results}')
        query = GetParameters.Request()
        query.names = ['inflation_layer.inflation_radius']
        values = result(node, getter.call_async(query)).values
        if len(values) != 1 or not math.isclose(values[0].double_value, radius, abs_tol=1e-9):
            raise RuntimeError(f'{name}: inflation read-back does not equal 0.05m')
        # StaticLayer can resize the grid to the map's resolution without
        # changing the configured resolution parameter. Do not mistake that
        # parameter for the current OccupancyGrid resolution.
        print(f'{name}: actual inflation_radius={values[0].double_value:.3f}m', flush=True)
    print('Both live radii verified at 0.05m. No goal or motion was sent. '
          'Enable motion before publishing a NEW goal.', flush=True)


if __name__ == '__main__':
    rclpy.init()
    node = Node('go2_set_inflation')
    try:
        apply(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
