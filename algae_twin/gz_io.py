"""Runtime control of the Gazebo world (spawn / remove / teleport / sprayer).

Primary path: the ros_gz_bridge service bridges (SpawnEntity / DeleteEntity /
SetEntityPose, available in ros_gz >= 1.0.11 on Jazzy). Operations queue until
the bridge is up; if it never appears, falls back to the `gz` CLI. The particle
emitter has no ROS bridge pair on Jazzy, so it is always toggled via `gz topic`.
"""
import math
import shlex
import subprocess
import threading

from geometry_msgs.msg import Pose

from .util import yaw_to_quat

_FALLBACK_WAIT_SEC = 20.0


class GzIo:
    def __init__(self, node, world='default'):
        self.node = node
        self.world = world
        self._queue = []
        self._lock = threading.Lock()
        self._elapsed = 0.0
        self._use_cli = False
        try:
            from ros_gz_interfaces.srv import (DeleteEntity, SetEntityPose,
                                               SpawnEntity)
            self._clients = {
                'spawn': node.create_client(SpawnEntity, f'/world/{world}/create'),
                'remove': node.create_client(DeleteEntity, f'/world/{world}/remove'),
                'pose': node.create_client(SetEntityPose, f'/world/{world}/set_pose'),
            }
        except ImportError:
            node.get_logger().warning(
                'ros_gz_interfaces not available — using gz CLI for world ops')
            self._clients = {}
            self._use_cli = True
        self._timer = node.create_timer(0.5, self._drain)

    # -- public API ----------------------------------------------------------
    def spawn(self, name, sdf, x, y, z=0.0, yaw=0.0):
        self._enqueue(('spawn', name, sdf, x, y, z, yaw))

    def remove(self, name):
        self._enqueue(('remove', name))

    def set_pose(self, name, x, y, yaw, z=0.0):
        self._enqueue(('pose', name, x, y, z, yaw))

    def set_emitter(self, topic, emitting):
        """Toggle a particle emitter (CLI only — no bridge pair on Jazzy)."""
        self._run_cli(['gz', 'topic', '-t', topic,
                       '-m', 'gz.msgs.ParticleEmitter',
                       '-p', f'emitting: {{data: {str(bool(emitting)).lower()}}}'])

    # -- queue / dispatch ------------------------------------------------------
    def _enqueue(self, op):
        with self._lock:
            self._queue.append(op)

    def _drain(self):
        self._elapsed += 0.5
        with self._lock:
            ops, self._queue = self._queue, []
        if not ops:
            return
        for op in ops:
            if not self._use_cli and self._clients[op[0]].service_is_ready():
                self._call_service(op)
            elif self._use_cli or self._elapsed > _FALLBACK_WAIT_SEC:
                if not self._use_cli:
                    self.node.get_logger().warning(
                        'gz service bridge not up after '
                        f'{_FALLBACK_WAIT_SEC}s — falling back to gz CLI')
                    self._use_cli = True
                self._call_cli(op)
            else:  # bridge probably still starting: try again next tick
                self._enqueue(op)

    def _report(self, op):
        def done(future):
            try:
                response = future.result()
                if not getattr(response, 'success', True):
                    self.node.get_logger().error(f'gz op failed: {op[:2]}')
            except Exception as err:  # service died mid-call
                self.node.get_logger().error(f'gz op error: {op[:2]}: {err}')
        return done

    def _call_service(self, op):
        kind = op[0]
        if kind == 'spawn':
            from ros_gz_interfaces.srv import SpawnEntity
            _, name, sdf, x, y, z, yaw = op
            request = SpawnEntity.Request()
            factory = request.entity_factory
            factory.name = name
            factory.sdf = sdf
            factory.allow_renaming = False
            factory.pose = _pose(x, y, z, yaw)
        elif kind == 'remove':
            from ros_gz_interfaces.msg import Entity
            from ros_gz_interfaces.srv import DeleteEntity
            request = DeleteEntity.Request()
            request.entity.name = op[1]
            request.entity.type = Entity.MODEL
        else:  # pose
            from ros_gz_interfaces.srv import SetEntityPose
            _, name, x, y, z, yaw = op
            request = SetEntityPose.Request()
            request.entity.name = name
            request.pose = _pose(x, y, z, yaw)
        future = self._clients[kind].call_async(request)
        future.add_done_callback(self._report(op))

    # -- gz CLI fallback -------------------------------------------------------
    def _call_cli(self, op):
        kind = op[0]
        if kind == 'spawn':
            _, name, sdf, x, y, z, yaw = op
            # escape backslash FIRST, then quotes, then control chars: protobuf
            # TextFormat terminates a quoted string at a raw newline, so the
            # multi-line SDF must have its newlines/tabs escaped or the CLI
            # fallback spawn silently fails to parse.
            sdf_escaped = (sdf.replace('\\', '\\\\').replace('"', '\\"')
                           .replace('\n', '\\n').replace('\t', '\\t')
                           .replace('\r', '\\r'))
            req = (f'sdf: "{sdf_escaped}" name: "{name}" '
                   f'pose: {{{_proto_pose_body(x, y, z, yaw)}}}')
            service, reqtype = f'/world/{self.world}/create', 'gz.msgs.EntityFactory'
        elif kind == 'remove':
            req = f'name: "{op[1]}" type: MODEL'
            service, reqtype = f'/world/{self.world}/remove', 'gz.msgs.Entity'
        else:  # gz.msgs.Pose carries the target entity in its `name` field
            _, name, x, y, z, yaw = op
            req = f'name: "{name}", {_proto_pose_body(x, y, z, yaw)}'
            service, reqtype = f'/world/{self.world}/set_pose', 'gz.msgs.Pose'
        self._run_cli(['gz', 'service', '-s', service, '--reqtype', reqtype,
                       '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
                       '--req', req])

    def _run_cli(self, cmd):
        logger = self.node.get_logger()

        def work():
            try:
                result = subprocess.run(cmd, capture_output=True, timeout=5)
                if result.returncode != 0:
                    logger.error(
                        f'gz CLI failed ({shlex.join(cmd[:4])}…): '
                        f'{result.stderr.decode(errors="replace").strip()}')
            except (subprocess.TimeoutExpired, FileNotFoundError) as err:
                logger.error(f'gz CLI unavailable: {err}')
        threading.Thread(target=work, daemon=True).start()


def _pose(x, y, z, yaw):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(x), float(y), float(z)
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = yaw_to_quat(yaw)
    return pose


def _proto_pose_body(x, y, z, yaw):
    half = yaw / 2.0
    return (f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{z: {math.sin(half)}, w: {math.cos(half)}}}')
