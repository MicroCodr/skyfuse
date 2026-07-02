"""aiohttp server: runs the simulation + fusion in real time and streams
the tactical picture to browsers over a WebSocket.

Client -> server messages:
    {"cmd": "toggle_sensor", "sensor": "radar", "enabled": false}

Server -> client (4 Hz): full world snapshot — truth, recent raw
detections, fused tracks with covariance, sensor status, and metrics.
"""
import asyncio
import json
import pathlib
import random
from collections import deque

from aiohttp import web, WSMsgType

from . import config, metrics
from .fusion.tracker import TrackManager
from .sensors import default_sensors
from .simulation import Simulation

WEB_DIR = pathlib.Path(__file__).resolve().parent.parent / 'web'
BROADCAST_PERIOD = 0.25
DETECTION_TRAIL = 4.0            # seconds of raw detections kept for display


class FusionServer:
    def __init__(self, seed=None):
        rng = random.Random(seed)
        self.sim = Simulation(seed)
        self.sensors = default_sensors(rng)
        self.tracker = TrackManager()
        self.clients = set()
        self.recent_dets = deque()

    # --- main loops --------------------------------------------------------

    async def sim_loop(self):
        while True:
            self.sim.step(config.SIM_DT)
            t = self.sim.time
            for sensor in self.sensors:
                if sensor.due(t):
                    dets = sensor.scan(t, self.sim.aircraft)
                    self.tracker.process_scan(t, dets)
                    for d in dets:
                        x, y = d.position()
                        self.recent_dets.append((t, d.sensor, x, y))
            while self.recent_dets and self.recent_dets[0][0] < t - DETECTION_TRAIL:
                self.recent_dets.popleft()
            await asyncio.sleep(config.SIM_DT)

    async def broadcast_loop(self):
        while True:
            if self.clients:
                msg = json.dumps(self.snapshot())
                await asyncio.gather(
                    *(ws.send_str(msg) for ws in list(self.clients)),
                    return_exceptions=True)
            await asyncio.sleep(BROADCAST_PERIOD)

    # --- snapshot ------------------------------------------------------------

    def snapshot(self):
        confirmed = self.tracker.confirmed
        return {
            'time': round(self.sim.time, 2),
            'truth': [
                {'id': ac.id, 'x': ac.x, 'y': ac.y,
                 'vx': ac.vx, 'vy': ac.vy, 'coop': ac.cooperative}
                for ac in self.sim.aircraft
            ],
            'detections': [
                {'t': round(t, 2), 'sensor': s, 'x': x, 'y': y}
                for (t, s, x, y) in self.recent_dets
            ],
            'tracks': [self._track_json(tr) for tr in self.tracker.tracks],
            'sensors': {
                s.name: {'enabled': s.enabled,
                         'pos': list(getattr(s, 'pos', (None, None)))}
                for s in self.sensors
            },
            'metrics': metrics.evaluate(confirmed, self.sim.aircraft),
        }

    @staticmethod
    def _track_json(tr):
        x = tr.ekf.x
        P = tr.ekf.P
        return {
            'id': tr.id,
            'x': x[0], 'y': x[1], 'vx': x[2], 'vy': x[3],
            'cov': [P[0, 0], P[0, 1], P[1, 1]],
            'status': tr.status.value,
            'hits': tr.hits,
            'sensors': dict(tr.sensor_counts),
        }

    # --- http ------------------------------------------------------------

    async def ws_handler(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self.clients.add(ws)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    self.handle_command(json.loads(msg.data))
        finally:
            self.clients.discard(ws)
        return ws

    def handle_command(self, cmd):
        if cmd.get('cmd') == 'toggle_sensor':
            for s in self.sensors:
                if s.name == cmd.get('sensor'):
                    s.enabled = bool(cmd.get('enabled'))

    async def index(self, request):
        return web.FileResponse(WEB_DIR / 'index.html')


def build_app(seed=None):
    server = FusionServer(seed)
    app = web.Application()
    app['server'] = server
    app.router.add_get('/', server.index)
    app.router.add_get('/ws', server.ws_handler)
    app.router.add_static('/static', WEB_DIR)

    async def start_tasks(app):
        app['tasks'] = [
            asyncio.create_task(server.sim_loop()),
            asyncio.create_task(server.broadcast_loop()),
        ]

    async def stop_tasks(app):
        for task in app['tasks']:
            task.cancel()

    app.on_startup.append(start_tasks)
    app.on_cleanup.append(stop_tasks)
    return app


def main(port=8777, seed=None):
    web.run_app(build_app(seed), port=port)
