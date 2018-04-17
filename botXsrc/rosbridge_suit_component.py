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

    def setup(self):
        command = 'roslaunch rosbridge_server rosbridge_websocket.launch'
        self.bridge_proc_id = external_command_pool.start_command(command)
        self.connected = False
        self.running = True
        self.thread_stopped = False
        self.q = queue.Queue()
        self.ws = WebSocket('ws://localhost:9090')
        self.monitor_t = Thread(target=self.monitor)
        self.monitor_t.start()

    def monitor(self):
        for event in persist(self.ws):
            try:
                if not self.running:
                    self.thread_stopped = True
                    return
                if event.name == 'connected':
                    print('responding to poll...')
                    self.connected = True
                elif event.name == 'pong':
                    print('received pong')
                    print(event)
                elif event.name == 'poll':
                    print('received polling')
                    if not self.q.empty():
                        req = self.q.get()
                        print('send ', req)
                        self.ws.send_text(req)
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

    def subscribe(self, topic, type):
        req = {
            'op': 'subscribe',
            'topic': topic,
            'type': type
        }
        self.q.put(json.dumps(req))

    def advertise(self, topic, type):
        req = {
            'op': 'advertise',
            'topic': topic,
            'type': type
        }
        self.q.put(json.dumps(req))
