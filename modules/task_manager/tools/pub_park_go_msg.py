from google.protobuf import text_format
from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time
import time
from modules.routing.proto.routing_pb2 import RoutingRequest, RoutingResponse
from modules.task_manager.proto.task_manager_pb2 import Task,ParkGOTask
if __name__ == '__main__':
    with open("/apollo/data/routing_.txt",'r') as  fread:
                routing_data=fread.read()
                rq_data=RoutingRequest()
                text_format.Parse(routing_data, rq_data)
    fread.close()
    cyber.init()
    task=Task()
    task.park_go_task.routing_request.CopyFrom(rq_data)
    task.park_go_task.stay_time=5
    task.task_type=2
    node = cyber.Node("replay_routing") 
    routingpub=node.create_writer("/apollo/task_manager",Task)
    print("success sent")
    while not cyber.is_shutdown():
        task.header.timestamp_sec=cyber_time.Time.now().to_sec()
        routingpub.write(task)
        a=input()
    cyber.shutdown()