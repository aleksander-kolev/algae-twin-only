"""Algae Twin operator UI — a browser dashboard served over the Python stdlib.

**No GUI library.** The node embeds a stdlib ``http.server`` and the operator
opens ``http://localhost:<port>`` in the browser every lab desktop already has
(it runs the Gazebo GUI + RViz). Neither ``python3-tk`` (Tkinter — no pip wheel,
needs an apt package + sudo) nor PyQt5 is required, so the operator console can
never be disabled for lack of a system package.

    ros2 run algae_twin operator_ui                 # live ROS dashboard
    ros2 run algae_twin operator_ui --port 8088     # override the HTTP port

Architecture: a ``SingleThreadedExecutor`` spins the ROS node on a daemon thread
and every subscription callback writes the thread-safe ``Store``; a
``ThreadingHTTPServer`` (bound to loopback) serves the page, pushes state to the
browser over Server-Sent Events (with a short-poll fallback), and turns command
POSTs into ROS publishers.
"""
import argparse
import base64
import json
import math
import os
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .ui_web import INDEX_HTML

DEFAULT_PORT = 8088
SSE_PERIOD = 0.1          # 10 Hz state push
MAX_SSE_STREAMS = 8       # cap long-lived streams; extra clients short-poll


class Store:
    """Thread-safe key/value state with per-key versions (UI redraw hints)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {}
        self._versions = {}

    def set(self, key, value):
        with self._lock:
            self._data[key] = value
            self._versions[key] = self._versions.get(key, 0) + 1

    def snapshot(self):
        with self._lock:
            snap = dict(self._data)
            snap['_versions'] = dict(self._versions)
            return snap


class RosLink:
    """All ROS plumbing for the UI: subscriptions into the Store, publishers out.

    rclpy is imported lazily here (kept out of the module-level imports).
    """

    def __init__(self, store):
        import rclpy
        from rclpy.node import Node  # noqa: F401  (documents the dependency)

        self.store = store
        rclpy.init(args=None)
        self.rclpy = rclpy
        self.node = rclpy.create_node('operator_ui')
        self._pub_lock = threading.Lock()   # serialise cross-thread command pubs
        self._make_publishers()
        self._make_subscriptions()
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        store.set('conn', 'ros')

    # -- outgoing ----------------------------------------------------------
    def _make_publishers(self):
        from geometry_msgs.msg import (PointStamped, PoseStamped,
                                       PoseWithCovarianceStamped)
        from std_msgs.msg import Bool, Empty, Float32, String

        from .util import latched_qos

        node = self.node
        self._pub = {
            'algae_add': node.create_publisher(PointStamped, '/algae/add', 10),
            'algae_remove': node.create_publisher(String, '/algae/remove', 10),
            'edit_add': node.create_publisher(String, '/edits/add', 10),
            'edit_remove': node.create_publisher(String, '/edits/remove', 10),
            'edit_clear': node.create_publisher(String, '/edits/clear', 10),
            'estop': node.create_publisher(Bool, '/estop', latched_qos()),
            'goto': node.create_publisher(PoseStamped, '/mission/goto', 10),
            'home': node.create_publisher(Empty, '/mission/home', 10),
            'recharge': node.create_publisher(Float32, '/sim/battery/set', 10),
            'resync': node.create_publisher(Empty, '/twin/resync', 10),
            'initialpose': node.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10),
        }

    def add_algae(self, x, y):
        from geometry_msgs.msg import PointStamped
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.point.x, msg.point.y = float(x), float(y)
        with self._pub_lock:
            self._pub['algae_add'].publish(msg)

    def remove_algae(self, algae_id):
        from std_msgs.msg import String
        with self._pub_lock:
            self._pub['algae_remove'].publish(String(data=str(algae_id)))

    def add_edit(self, cx, cy, size_x, size_y):
        from .util import json_msg
        with self._pub_lock:
            self._pub['edit_add'].publish(json_msg({
                'cx': float(cx), 'cy': float(cy),
                'size_x': float(size_x), 'size_y': float(size_y),
                'yaw': 0.0, 'source': 'operator'}))

    def remove_edit(self, edit_id):
        from .util import json_msg
        with self._pub_lock:
            self._pub['edit_remove'].publish(json_msg({'id': str(edit_id)}))

    def clear_edits(self, source):
        from .util import json_msg
        with self._pub_lock:
            self._pub['edit_clear'].publish(json_msg({'source': str(source)}))

    def set_estop(self, engaged):
        from std_msgs.msg import Bool
        with self._pub_lock:
            self._pub['estop'].publish(Bool(data=bool(engaged)))

    def goto(self, x, y, yaw):
        from geometry_msgs.msg import PoseStamped

        from .util import yaw_to_quat
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = yaw_to_quat(float(yaw))
        with self._pub_lock:
            self._pub['goto'].publish(msg)

    def go_home(self):
        from std_msgs.msg import Empty
        with self._pub_lock:
            self._pub['home'].publish(Empty())

    def resync_twin(self):
        from std_msgs.msg import Empty
        with self._pub_lock:
            self._pub['resync'].publish(Empty())

    def recharge(self, percent=100.0):
        from std_msgs.msg import Float32
        with self._pub_lock:
            self._pub['recharge'].publish(Float32(data=float(percent)))

    def set_pose(self, x, y, yaw):
        from geometry_msgs.msg import PoseWithCovarianceStamped

        from .util import yaw_to_quat
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = \
            yaw_to_quat(float(yaw))
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        with self._pub_lock:
            self._pub['initialpose'].publish(msg)

    # -- incoming ----------------------------------------------------------
    def _make_subscriptions(self):
        from nav_msgs.msg import OccupancyGrid, Path
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String

        from .util import latched_qos, parse_json, sensor_qos

        node, store = self.node, self.store

        def on_map(msg):
            store.set('map', {
                'w': msg.info.width, 'h': msg.info.height,
                'res': msg.info.resolution,
                'ox': msg.info.origin.position.x,
                'oy': msg.info.origin.position.y,
                'cells': bytes((v + 256) % 256 for v in msg.data),
            })

        def on_json(key):
            def cb(msg):
                payload = parse_json(msg, node.get_logger(), key)
                if payload is not None:
                    store.set(key, payload)
            return cb

        def on_plan(msg):
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            store.set('plan', pts[::max(1, len(pts) // 200)])

        def on_scan(msg):
            status = store.snapshot().get('status') or {}
            pose = (status.get('real') or {}).get('pose')
            if not pose:
                pose = (status.get('sim') or {}).get('pose')
            if not pose:
                return
            px, py, pyaw = pose
            pts = []
            angle = msg.angle_min
            for i, rng in enumerate(msg.ranges):
                if i % 2 == 0 and msg.range_min < rng < msg.range_max:
                    a = pyaw + angle
                    pts.append((px + rng * math.cos(a), py + rng * math.sin(a)))
                angle += msg.angle_increment
            store.set('scan', pts)

        node.create_subscription(OccupancyGrid, '/map', on_map, latched_qos())
        node.create_subscription(String, '/ui/status', on_json('status'), 10)
        node.create_subscription(String, '/algae/state', on_json('algae'),
                                 latched_qos())
        node.create_subscription(String, '/edits/state', on_json('edits'),
                                 latched_qos())
        node.create_subscription(Path, '/plan', on_plan, 10)
        node.create_subscription(LaserScan, '/scan', on_scan, sensor_qos())
        node.create_subscription(
            LaserScan, '/sim/scan',
            lambda m: None
            if (store.snapshot().get('status') or {}).get('mode') == 'twin'
            else on_scan(m),
            sensor_qos())

    def shutdown(self):
        self._executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.rclpy.try_shutdown()


# -- command dispatch (HTTP body -> RosLink method) --------------------------
def _num(value):
    """Validate a finite float at the HTTP boundary (rejects NaN/inf/garbage)."""
    out = float(value)
    if not math.isfinite(out):
        raise ValueError('non-finite number')
    return out


def dispatch_command(link, name, body):
    """Map a /cmd/<name> POST body to the matching link call. Raises on bad input."""
    if name == 'algae_add':
        link.add_algae(_num(body['x']), _num(body['y']))
    elif name == 'algae_remove':
        link.remove_algae(str(body['id']))
    elif name == 'edit_add':
        link.add_edit(_num(body['cx']), _num(body['cy']),
                      _num(body['size_x']), _num(body['size_y']))
    elif name == 'edit_remove':
        link.remove_edit(str(body['id']))
    elif name == 'clear_edits':
        source = str(body.get('source', 'all'))
        if source not in ('all', 'operator', 'mirrored'):
            raise ValueError(f'bad source {source!r}')
        link.clear_edits(source)
    elif name == 'estop':
        link.set_estop(bool(body['engaged']))
    elif name == 'goto':
        link.goto(_num(body['x']), _num(body['y']), _num(body['yaw']))
    elif name == 'home':
        link.go_home()
    elif name == 'resync':
        link.resync_twin()
    elif name == 'recharge':
        link.recharge(_num(body.get('percent', 100.0)))
    elif name == 'setpose':
        link.set_pose(_num(body['x']), _num(body['y']), _num(body['yaw']))
    else:
        raise KeyError(name)


# -- HTTP server -------------------------------------------------------------
class _Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    # silence the default per-request stderr logging (it floods the console)
    def log_message(self, *args):
        return

    @property
    def _store(self):
        return self.server.store

    @property
    def _link(self):
        return self.server.link

    def _send(self, code, body=b'', ctype='text/plain'):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        if body:
            self.wfile.write(body)

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self._send(200, INDEX_HTML.encode('utf-8'), 'text/html; charset=utf-8')
        elif path == '/map.json':
            self._send_map()
        elif path == '/state.json':
            body = json.dumps(dynamic_state(self._store), default=_json_default)
            self._send(200, body.encode('utf-8'), 'application/json')
        elif path == '/events':
            self._stream_events()
        else:
            self._send(404, b'not found')

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        if not path.startswith('/cmd/'):
            self._send(404, b'not found')
            return
        name = path[len('/cmd/'):]
        try:
            length = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(length) if length else b'{}'
            body = json.loads(raw or b'{}')
            dispatch_command(self._link, name, body)
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as err:
            self._send(400, f'bad command: {err}'.encode('utf-8'))
            return
        self._send(204)

    def _send_map(self):
        grid = self._store.snapshot().get('map')
        if not grid:
            self._send(204)        # map not received yet
            return
        payload = {
            'w': grid['w'], 'h': grid['h'], 'res': grid['res'],
            'ox': grid['ox'], 'oy': grid['oy'],
            'cells_b64': base64.b64encode(grid['cells']).decode('ascii'),
        }
        self._send(200, json.dumps(payload).encode('utf-8'), 'application/json')

    def _stream_events(self):
        if not self.server.acquire_stream():
            self._send(503, b'too many streams - falling back to poll')
            return
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            while not self.server.stopping:
                payload = json.dumps(dynamic_state(self._store), default=_json_default)
                self.wfile.write(f'data: {payload}\n\n'.encode('utf-8'))
                self.wfile.flush()
                time.sleep(SSE_PERIOD)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass               # browser tab closed; free the worker thread
        finally:
            self.server.release_stream()


def _json_default(obj):
    if isinstance(obj, (bytes, bytearray)):
        return base64.b64encode(obj).decode('ascii')
    raise TypeError(f'not serialisable: {type(obj)}')


def dynamic_state(store):
    """Snapshot for the live stream — everything except the bulky map cells
    (the browser fetches /map.json once and refetches only when map_version
    changes), so each 10 Hz frame stays a few KB."""
    snap = store.snapshot()
    out = {k: v for k, v in snap.items() if k != 'map'}
    out['map_version'] = snap.get('_versions', {}).get('map')
    return out


class UiServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, store, link):
        super().__init__(addr, _Handler)
        self.store = store
        self.link = link
        self.stopping = False
        self._streams = 0
        self._stream_lock = threading.Lock()

    def acquire_stream(self):
        with self._stream_lock:
            if self._streams >= MAX_SSE_STREAMS:
                return False
            self._streams += 1
            return True

    def release_stream(self):
        with self._stream_lock:
            self._streams = max(0, self._streams - 1)

    def handle_error(self, request, client_address):
        # a browser tab disconnecting (keep-alive abort, SSE close) is normal —
        # don't spew a traceback to the operator console; surface real errors.
        err = sys.exc_info()[1]
        if isinstance(err, (ConnectionError, OSError)):
            return
        super().handle_error(request, client_address)


def _serve(store, link, host, port):
    """Bind the first free port at/after `port`, start serving, return the server."""
    last_err = None
    for candidate in range(port, port + 20):
        try:
            server = UiServer((host, candidate), store, link)
            break
        except OSError as err:
            last_err = err
    else:
        raise RuntimeError(f'no free port in {port}..{port + 19}: {last_err}')
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description='Algae Twin operator UI (browser)')
    parser.add_argument('--port', type=int, default=DEFAULT_PORT,
                        help=f'HTTP port (default {DEFAULT_PORT})')
    parser.add_argument('--host',
                        default=os.environ.get('ALGAE_UI_HOST', '127.0.0.1'),
                        help='bind address (default 127.0.0.1; 0.0.0.0 exposes '
                             'the UI on the network — use only in Docker/trusted nets)')
    parser.add_argument('--no-browser', action='store_true',
                        help='do not auto-open the browser')
    args, _ = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    store = Store()
    link = RosLink(store)

    server = _serve(store, link, args.host, args.port)
    url = f'http://localhost:{server.server_address[1]}'
    banner = (f"\n  Algae Twin operator UI -> {url}\n"
              f"  (open it in a browser; Ctrl+C here to stop)\n")
    print(banner, flush=True)
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass               # headless / no browser: the URL is printed above

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        server.stopping = True
        server.shutdown()
        link.shutdown()


if __name__ == '__main__':
    main()
