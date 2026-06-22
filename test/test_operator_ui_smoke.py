"""ROS-free smoke test for the browser operator UI (twin-only build).

Drives operator_ui's stdlib HTTP server with a FAKE link (no ROS, and no
demo_link — that no-hardware preview path is stripped from this build) to verify
the page, the map/state endpoints, the SSE stream, the command round-trip
(POST -> link -> Store) and boundary input validation. Runs anywhere with plain
Python 3 (operator_ui only imports rclpy lazily, inside RosLink).

    python test/test_operator_ui_smoke.py
"""
import json
import os
import sys
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from algae_twin.operator_ui import Store, _serve          # noqa: E402


class FakeLink:
    """Stand-in for RosLink: records commands into the Store, no ROS required."""

    def __init__(self, store):
        self.store = store
        self._lock = threading.Lock()
        store.set('conn', 'test')
        store.set('map', {'w': 4, 'h': 3, 'res': 0.05, 'ox': 0.0, 'oy': 0.0,
                          'cells': bytes([0, 0, 100, 255] * 3)})
        store.set('status', {'mode': 'twin', 'estop': False,
                             'real': {'pose': [0, 0, 0], 'ok': True},
                             'sim': {'pose': [0, 0, 0], 'ok': True}})
        store.set('algae', {'algae': []})

    def add_algae(self, x, y):
        with self._lock:
            data = dict(self.store.snapshot().get('algae', {'algae': []}))
            data['algae'] = list(data.get('algae', [])) + [{'x': x, 'y': y}]
            self.store.set('algae', data)

    def set_estop(self, engaged):
        status = dict(self.store.snapshot().get('status', {}))
        status['estop'] = bool(engaged)
        self.store.set('status', status)

    # the remaining commands are exercised for dispatch/validation, not state
    def remove_algae(self, _i): pass
    def add_edit(self, *_a): pass
    def remove_edit(self, _i): pass
    def clear_edits(self, _s): pass
    def goto(self, *_a): pass
    def go_home(self): pass
    def recharge(self, *_a): pass
    def set_pose(self, *_a): pass
    def shutdown(self): pass


def _get(base, path):
    with urllib.request.urlopen(base + path, timeout=3) as resp:
        return resp.status, resp.read()


def _post(base, path, body):
    req = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=3) as resp:
        return resp.status


def test_operator_ui_smoke():
    store = Store()
    link = FakeLink(store)
    server = _serve(store, link, '127.0.0.1', 8097)
    base = f'http://127.0.0.1:{server.server_address[1]}'
    try:
        # index page renders
        status, body = _get(base, '/')
        assert status == 200 and b'Algae Twin' in body

        # map served as base64 cells with sane geometry
        status, body = _get(base, '/map.json')
        assert status == 200
        mp = json.loads(body)
        assert mp['w'] == 4 and mp['h'] == 3 and mp['cells_b64']

        # live state excludes the bulky map, carries a version + twin mode
        status, body = _get(base, '/state.json')
        st = json.loads(body)
        assert 'map' not in st, 'map cells must not be in the live stream'
        assert st.get('map_version') is not None
        assert st['status']['mode'] == 'twin'

        # SSE stream emits data frames
        with urllib.request.urlopen(base + '/events', timeout=3) as resp:
            chunk = resp.read(32)
        assert chunk.startswith(b'data: '), chunk[:16]

        # command round-trip: place algae -> appears in the Store
        n0 = len(store.snapshot()['algae']['algae'])
        assert _post(base, '/cmd/algae_add', {'x': 0.5, 'y': -0.5}) == 204
        time.sleep(0.2)
        assert len(store.snapshot()['algae']['algae']) == n0 + 1

        # boundary validation: garbage input is rejected with 400
        rejected = False
        try:
            _post(base, '/cmd/algae_add', {'x': 'not-a-number', 'y': 0.0})
        except urllib.error.HTTPError as err:
            rejected = err.code == 400
        assert rejected, 'non-finite/garbage input must 400'

        # estop round-trips through the link into the status snapshot
        assert _post(base, '/cmd/estop', {'engaged': True}) == 204
        time.sleep(0.2)
        assert store.snapshot()['status']['estop'] is True
    finally:
        server.stopping = True
        server.shutdown()
        link.shutdown()


if __name__ == '__main__':
    test_operator_ui_smoke()
    print('UI SMOKE TEST PASS')
