"""Actual ROS route-file commands and Nav2 actions; no robot or real planner."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import uuid

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Empty, String
import yaml

from navigation_queue_smoke import QueueEndpoints, Operator, PoseStamped, wait_for
from route_store import RouteStore


def map_fixture(directory):
    directory.mkdir()
    (directory/'map.pcd').write_text('VERSION .7\nFIELDS x y z\nDATA ascii\n0 0 0\n1 0 0\n')
    (directory/'nav.pgm').write_bytes(b'P5\n50 50\n255\n'+bytes([254])*2500)
    (directory/'nav.yaml').write_text(yaml.safe_dump(dict(image='nav.pgm',resolution=.1,
        origin=[-2.,-2.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
    (directory/'metadata.yaml').write_text(yaml.safe_dump(dict(
        parameters=dict(map_name='test_farm'),ground_model=dict(plane=[0.,0.,1.,0.]),
        body_height_above_ground=.3,
        map_start_pose=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,.3],[0.,0.,0.,1.]],
        map_from_source_map=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])))


def main():
    with tempfile.TemporaryDirectory(prefix='offline-route-') as tmp:
        root=Path(tmp)
        map_fixture(root/'navigation_map')
        rclpy.init(args=['--ros-args','-p','mode:=navigation','-p',
                        f'route_map_directory:={root / "navigation_map"}'])
        operator,endpoints=Operator(),QueueEndpoints()
        executor=MultiThreadedExecutor(num_threads=5)
        executor.add_node(operator)
        executor.add_node(endpoints)
        replies={}
        commands=endpoints.create_publisher(String,'/navigation/route/command',10)
        subscription=endpoints.create_subscription(String,'/navigation/route/result',
            lambda msg: replies.setdefault(json.loads(msg.data)['request_id'],[]).append(json.loads(msg.data)),10)
        spinner=threading.Thread(target=executor.spin,daemon=True)
        spinner.start()

        def cli(*arguments):
            executable=Path(__file__).resolve().parents[1]/'go2_3d/route_cli.py'
            result=subprocess.run([sys.executable,str(executable),'--directory',
                str(root/'routes'),*arguments],capture_output=True,text=True,timeout=25)
            assert result.returncode==0,(result.stdout,result.stderr)
            return result.stdout

        def ask(command,route=None,request=None):
            request=request or dict(request_id=str(uuid.uuid4()),command=command)
            if route is not None:
                request['route']=copy.deepcopy(route)
            before=len(replies.get(request['request_id'],[]))
            commands.publish(String(data=json.dumps(request)))
            wait_for(lambda: len(replies.get(request['request_id'],[]))>before)
            return replies[request['request_id']][-1],request

        def ready():
            wait_for(lambda: operator.localized() and len(operator.grids)==3
                and operator.action.server_is_ready() and operator.parameters.service_is_ready()
                and operator.motion_state.service_is_ready())
            wait_for(lambda: operator.buffer.can_transform('map','base_footprint',rclpy.time.Time()))
            wait_for(lambda: commands.get_subscription_count()==1)

        def cancel():
            endpoints.cancel.publish(Empty())
            wait_for(lambda: not operator.waypoints and operator.goal_handle is None
                and not operator.goal_pending and not operator.cancelling and not operator.route_paused)

        def finish(goal_number):
            wait_for(lambda: endpoints.goals==goal_number and operator.goal_handle is not None)
            endpoints.outcomes[goal_number]=4

        try:
            endpoints.valid=True
            ready()
            for invalid_id in (None, 42, float('nan'), [], {}):
                before=len(replies.get('',[]))
                commands.publish(String(data=json.dumps(dict(request_id=invalid_id,command='snapshot'))))
                wait_for(lambda: len(replies.get('',[]))>before)
                assert not replies[''][-1]['success'],'Malformed request ID was accepted'
            for number,x in enumerate((.4,.8,1.2),1):
                pose=PoseStamped()
                pose.header.frame_id='map'
                pose.pose.position.x=x
                pose.pose.orientation.w=1.
                endpoints.goal.publish(pose)
                wait_for(lambda: len(operator.waypoints)==number)
            snapshot,_=ask('snapshot')
            assert snapshot['success'],snapshot
            saved=snapshot['route']
            assert [p['position'][0] for p in saved['waypoints']]==[.4,.8,1.2]
            store=RouteStore(root/'routes')
            assert 'SAVED farm_patrol: 3 points' in cli('save','farm_patrol')
            stored=root/'routes/farm_patrol.json'
            assert store.load('farm_patrol')==saved
            assert json.loads(cli('show','farm_patrol'))==saved
            assert 'farm_patrol: 3 points' in cli('list')
            original_bytes=stored.read_bytes()
            cancel()
            assert ask('snapshot')[0]['route']==saved,'Cancel lost reusable plan history'

            # Full validation happens before any queue mutation or motion enable.
            wrong=copy.deepcopy(saved)
            wrong['map']['fingerprint']='0'*64
            result,_=ask('load',wrong)
            assert not result['success'] and not operator.waypoints,result
            wrong=copy.deepcopy(saved)
            wrong['waypoints'][1]['position'][0]=99.
            result,_=ask('load',wrong)
            assert not result['success'] and not operator.waypoints,result
            assert endpoints.goals==0 and not endpoints.get_parameter('enable_control').value

            # Loading queues the route but never enables control or dispatches.
            assert 'LOADED farm_patrol' in cli('load','farm_patrol')
            time.sleep(.3)
            assert len(operator.waypoints)==3 and endpoints.goals==0
            assert not endpoints.get_parameter('enable_control').value
            cancel()
            result,load_request=ask('load',store.load('farm_patrol'))
            assert result['success'] and not result['start_requested'],result
            time.sleep(.3)
            assert len(operator.waypoints)==3 and endpoints.goals==0
            assert not endpoints.get_parameter('enable_control').value
            assert ask('load',saved)[0]['success'] is False,'Load replaced an existing queue'
            duplicate,_=ask('load',request=load_request)
            assert duplicate==result and len(operator.waypoints)==3
            collision=dict(load_request,command='run')
            assert not ask('run',request=collision)[0]['success'],'Reused ID accepted different payload'
            cancel()

            # Explicit run uses the existing enable handshake and executes once.
            result,run_request=ask('run',saved)
            assert result['success'] and result['start_requested'],result
            for goal in (1,2,3):
                finish(goal)
            wait_for(lambda: operator.completed_waypoints==3 and operator.goal_handle is None)
            assert endpoints.positions==[.4,.8,1.2] and endpoints.max_active==1
            assert ask('snapshot')[0]['route']==saved,'Completed points disappeared from saved plan'
            assert ask('run',request=run_request)[0]==result
            time.sleep(.3)
            assert endpoints.goals==3,'Retransmitted run restarted a completed patrol'
            print('PASS offline route snapshot/full history, atomic map/point rejection, explicit three-point run and idempotent retries')

            # Bridge is still enabled after success: load must still remain idle.
            assert endpoints.get_parameter('enable_control').value
            assert ask('load',saved)[0]['success']
            time.sleep(.3)
            assert endpoints.goals==3 and not operator.route_started
            cancel()
            assert ask('run',saved)[0]['success']
            wait_for(lambda: endpoints.goals==4 and operator.goal_handle is not None)
            assert not ask('load',saved)[0]['success'],'Load replaced an active route'
            endpoints.enable.publish(Bool(data=False))
            wait_for(lambda: endpoints.cancels==1 and operator.goal_handle is None and not operator.waypoints)
            wait_for(lambda: not endpoints.get_parameter('enable_control').value)
            assert ask('snapshot')[0]['route']==saved
            assert stored.read_bytes()==original_bytes,'Stopping altered the saved file'

            # A genuine Nav2 failure is retained, not reported as a completed point.
            assert ask('run',saved)[0]['success']
            wait_for(lambda: endpoints.goals==5 and operator.goal_handle is not None)
            endpoints.outcomes[5]=6
            wait_for(lambda: operator.route_paused and operator.goal_handle is None)
            assert operator.completed_waypoints==0
            assert [p.pose.position.x for _,p in operator.waypoints]==[.4,.8,1.2]
            assert not ask('run',saved)[0]['success'],'Run bypassed the paused failed route'
            endpoints.resume.publish(Empty())
            for goal in (6,7,8):
                finish(goal)
            wait_for(lambda: operator.completed_waypoints==3 and operator.goal_handle is None)
            assert endpoints.positions[4:]==[.4,.4,.8,1.2]
            print('PASS load remains idle with enabled bridge; manual stop preserves route file; real failure pauses and resume retries the same point')

            # Restart the real Operator node: RAM is gone, the file survives.
            executor.remove_node(operator)
            operator.destroy_node()
            operator=Operator()
            executor.add_node(operator)
            ready()
            assert not ask('snapshot')[0]['success'],'A restarted node silently retained a route'
            saved_again=RouteStore(root/'routes').load('farm_patrol')
            assert saved_again==saved
            assert 'START REQUESTED farm_patrol' in cli('run','farm_patrol')
            for goal in (9,10,11):
                finish(goal)
            wait_for(lambda: operator.completed_waypoints==3 and operator.goal_handle is None)
            assert endpoints.positions[-3:]==[.4,.8,1.2]
            assert stored.read_bytes()==original_bytes
            print('PASS real CLI save/show/list/load/run, persisted route reload and three-point execution after Operator restart; no external network or robot motion')
        finally:
            endpoints.accept_wait.set()
            executor.shutdown()
            spinner.join(timeout=3)
            operator.destroy_node()
            endpoints.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    main()
