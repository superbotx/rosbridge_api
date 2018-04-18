from botX.components import BaseComponent
from botX.applications import external_command_pool
from lomond.websocket import WebSocket
from lomond.persist import persist
from threading import Thread
import json
import queue
import time
import logging

class RosbridgeSuitComponent(BaseComponent):

    def __init__(self):
        super(RosbridgeSuitComponent, self).__init__()
        self.is_setup = False
        self.connected = False
        self.running = True
        self.pub_callbacks = {}
        self.srv_callbacks = {}
        self.thread_stopped = False
        self.q = queue.Queue()

    def setup(self):
        if self.is_setup:
            print('setup is done earlier, return')
            return
        command = 'roslaunch rosbridge_server rosbridge_websocket.launch'
        self.bridge_proc_id = external_command_pool.start_command(command)
        self.ws = WebSocket('ws://localhost:9090')
        self.monitor_t = Thread(target=self.monitor)
        self.monitor_t.start()
        self.is_setup = True

    def monitor(self):
        for event in persist(self.ws):
            try:
                if not self.running:
                    self.thread_stopped = True
                    return
                if event.name == 'connecting':
                    print('connecting...')
                elif event.name == 'connected':
                    print('connected')
                    self.connected = True
                elif event.name == 'pong':
                    print('received pong')
                elif event.name == 'poll':
                    print('received poll')
                elif event.name == 'text':
                    json_str = event.text
                    data = json.loads(json_str)
                    if data['op'] == 'publish':
                        topic_id = data['topic']
                        if topic_id in self.pub_callbacks:
                            self.pub_callbacks[topic_id](data['msg'])
                    elif data['op'] == 'service_response':
                        service_id = data['service']
                        if service_id in self.srv_callbacks:
                            self.srv_callbacks[service_id](data['result'])
                    else:
                        print('unknown data: ', data)
                else:
                    print('unhandled event: ', event.name)
                    print(event)
            except:
                logging.exception('error handling %r', event)

    def shutdown(self):
        self.running = False
        while not self.thread_stopped:
            time.sleep(1)
        external_command_pool.end_command(self.bridge_proc_id)

    def send_req(self, req):
        while not self.connected:
            time.sleep(1)
        self.ws.send_text(json.dumps(req))

    def subscribe(self, topic, type, callback):
        req = {
            'op': 'subscribe',
            'topic': topic,
            'type': type
        }
        self.pub_callbacks[topic] = callback
        self.send_req(req)

    def unsubscribe(self, topic):
        req = {
            'op': 'unsubscribe',
            'topic': topic
        }
        del self.pub_callbacks[topic]
        self.send_req(req)

    def publish(self, topic, msg):
        req = {
            'op': 'publish',
            'topic': topic,
            'msg': msg
        }
        self.send_req(req)

    def call_service(self, service, callback, args=[]):
        req = {
            'op': 'call_service',
            'service': service,
            'args': args
        }
        self.srv_callbacks[service] = callback
        self.send_req(req)

    def respond_service(self, service, result):
        req = {
            'op': 'service_response',
            'service': service,
            'result': result
        }
        self.send_req(req)

    def advertise(self, topic, type):
        req = {
            'op': 'advertise',
            'topic': topic,
            'type': type
        }
        self.send_req(req)

    def advertise_service(self, type, service):
        req = {
            'op': 'advertise_service',
            'type': type,
            'service': service
        }
        self.send_req(req)

    def unadvertise(self, topic):
        req = {
            'op': 'unadvertise',
            'topic': topic
        }
        self.send_req(req)

    def unadvertise_service(self, service):
        req = {
            'op': 'unadvertise_service',
            'service': service
        }
        self.send_req(req)
