#!/usr/bin/env python3
"""
twin_host  (NEW NODE - vector_twin_tools/twin_host.py)

Lets the VECTOR twin run on ANY computer (Windows, macOS, Linux without ROS).

Runs on the LAB PC (Ubuntu + ROS 2 + ~/VECTOR built). A twin app on another
computer asks it over the lab network to start the ROS programs for a mode,
and gets its OWN ros_tcp_endpoint port to connect Unity to (one endpoint can
only serve one Unity at a time).

    Viewer PC (Unity only)                     Lab PC (this program)
    menu "ROS on lab PC"  --HTTP :10100-->  starts fake_fleet / serial_bridge / wheel
    twin  ------------- TCP :10001.. ---->  that viewer's own ros_tcp_endpoint

Mode programs are SHARED: a second viewer asking for the same mode just gets
its own endpoint and watches the same cars. When the last viewer leaves
(Esc / quit / crash = no ping for --viewer-timeout s) the programs stop.

SAFETY: a viewer can only choose a mode, numbers and IP addresses - never a
command. The steering-wheel command is set HERE with --wheel-cmd.

HTTP API (JSON):
  GET  /status[?viewer=v1]
  POST /start  {"mode":"fleet"|"real"|"wheel", "fleet_cars":3, "fleet_speed":0.4,
                "cars":[{"id":1,"ip":"192.168.1.148"}], "start_wheel":true}
               -> {"ok":true, "viewer":"v1", "port":10001}
  POST /ping   {"viewer":"v1"}
  POST /stop   {"viewer":"v1"}
  GET  /test?ip=192.168.1.148     can THIS PC reach the Pi's telemetry port?

Run (normally through Launcher/start_twin_host.sh, which also sources ROS):
  ros2 run vector_twin_tools twin_host
"""

import argparse
import ipaddress
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HOME = os.path.expanduser('~')
LOG_DIR = os.path.join(HOME, '.vector_twin', 'host_logs')


def log(msg):
    print(time.strftime('[%H:%M:%S] ') + msg, flush=True)


# --------------------------------------------------------------------------- programs
class Program:
    """One background program in its own process group (so Ctrl+C reaches its children)."""

    def __init__(self, name, cmd):
        self.name = name
        self.cmd = cmd
        os.makedirs(LOG_DIR, exist_ok=True)
        self.log_path = os.path.join(LOG_DIR, name + '.log')
        self._log = open(self.log_path, 'w')
        env = dict(os.environ, PYTHONUNBUFFERED='1')
        self.proc = subprocess.Popen(cmd, stdout=self._log, stderr=subprocess.STDOUT,
                                     start_new_session=True, env=env)
        log(f'started {name} (pid {self.proc.pid}): {" ".join(cmd)}')

    def running(self):
        return self.proc.poll() is None

    def code(self):
        rc = self.proc.poll()
        return 0 if rc is None else rc

    def stop(self):
        if self.running():
            try:
                os.killpg(self.proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)   # leftovers of the group
        except ProcessLookupError:
            pass
        try:
            self.proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass
        self._log.close()
        log(f'stopped {self.name}')

    def last_line(self):
        try:
            with open(self.log_path, 'rb') as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 2048))
                lines = [l.strip() for l in f.read().decode('utf-8', 'replace').splitlines() if l.strip()]
                return lines[-1][:120] if lines else ''
        except OSError:
            return ''

    def status(self):
        return {'name': self.name, 'running': self.running(), 'code': self.code(),
                'last': '' if self.running() else self.last_line()}


# --------------------------------------------------------------------------- host state
class TwinHost:
    def __init__(self, args):
        self.a = args
        self.lock = threading.Lock()
        self.viewers = {}          # viewer id -> {'port', 'endpoint': Program, 'last_ping', 'addr'}
        self.mode_key = None       # what the shared programs are running for
        self.mode_desc = ''
        self.programs = []         # shared mode programs
        self.next_viewer = 1

    # ---------- validation ----------
    @staticmethod
    def _ip(value):
        ip = str(value).strip()
        ipaddress.IPv4Address(ip)  # raises ValueError if not a plain IPv4 address
        return ip

    def _parse_request(self, req):
        mode = req.get('mode')
        if mode not in ('fleet', 'real', 'wheel'):
            raise ValueError("mode must be 'fleet', 'real' or 'wheel'")
        out = {'mode': mode}
        if mode == 'fleet':
            n = int(req.get('fleet_cars', 3))
            speed = float(req.get('fleet_speed', 0.4))
            if not 1 <= n <= 20:
                raise ValueError('fleet_cars must be 1..20')
            if not 0.05 <= speed <= 3.0:
                raise ValueError('fleet_speed must be 0.05..3.0 m/s')
            out['fleet_cars'] = n
            out['fleet_speed'] = round(speed, 3)
        else:
            cars = req.get('cars') or []
            if not 1 <= len(cars) <= 10:
                raise ValueError('give 1..10 cars')
            seen, clean = set(), []
            for c in cars:
                cid = int(c.get('id'))
                if not 1 <= cid <= 99:
                    raise ValueError('car id must be 1..99')
                if cid in seen:
                    raise ValueError(f'car id {cid} used twice')
                seen.add(cid)
                clean.append({'id': cid, 'ip': self._ip(c.get('ip'))})
            out['cars'] = sorted(clean, key=lambda c: c['id'])
            out['start_wheel'] = bool(req.get('start_wheel', False)) and mode == 'wheel'
        return out

    # ---------- shared mode programs ----------
    def _ns(self, cid):
        return self.a.namespace_format.format(cid)

    def _start_mode(self, m):
        key = json.dumps(m, sort_keys=True)
        if key == self.mode_key and all(p.running() for p in self.programs):
            return  # already running exactly this - just share it
        self._stop_mode()

        if m['mode'] == 'fleet':
            self.programs.append(Program('fake_fleet', [
                'ros2', 'run', 'vector_twin_tools', 'fake_fleet_publisher', '--ros-args',
                '-p', f"num_cars:={m['fleet_cars']}",
                '-p', f"speed_mps:={float(m['fleet_speed']):.3f}"]))
            self.mode_desc = f"fleet test, {m['fleet_cars']} cars"
        else:
            for c in m['cars']:
                ns = self._ns(c['id'])
                name = 'serial_bridge_' + ns.strip('/').replace('/', '_')
                self.programs.append(Program(name, [
                    'ros2', 'run', 'vector_twin_tools', 'serial_bridge', '--ros-args',
                    '-r', f'__node:={name}',
                    '-p', f"port:=socket://{c['ip']}:{self.a.telemetry_port}",
                    '-r', f'/f1r3/car/telemetry:={ns}/car/telemetry',
                    '-r', f'/f1r3/car/calibrating:={ns}/car/calibrating',
                    '-r', f'/f1r3/car/telemetry_raw:={ns}/car/telemetry_raw',
                    '-r', f'/f1r3/car/cmd:={ns}/car/cmd']))
            if m.get('start_wheel'):
                ws = os.path.expanduser(self.a.wheel_ws)
                setup = os.path.join(ws, 'install', 'setup.bash')
                if os.path.isfile(setup):
                    self.programs.append(Program('wheel', [
                        'bash', '-c', f'source "{setup}" && exec {self.a.wheel_cmd}']))
                else:
                    log(f'WARNING: wheel workspace not built ({setup}) - wheel not started')
            ids = ', '.join(f"{c['id']:02d}" for c in m['cars'])
            self.mode_desc = ('steering wheel' if m['mode'] == 'wheel' else 'real car') + f', car {ids}'
        self.mode_key = key
        log(f'mode -> {self.mode_desc}')

    def _stop_mode(self):
        for p in self.programs:
            p.stop()
        self.programs = []
        self.mode_key = None
        self.mode_desc = ''

    # ---------- viewers ----------
    def _free_port(self):
        used = {v['port'] for v in self.viewers.values()}
        for port in range(self.a.first_endpoint_port, self.a.first_endpoint_port + self.a.max_viewers):
            if port in used:
                continue
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    s.bind(('0.0.0.0', port))
                    return port
                except OSError:
                    continue
        return None

    def start(self, req, addr):
        m = self._parse_request(req)
        with self.lock:
            port = self._free_port()
            if port is None:
                raise RuntimeError(f'all {self.a.max_viewers} viewer slots are in use')
            self._start_mode(m)
            vid = f'v{self.next_viewer}'
            self.next_viewer += 1
            ep = Program(f'endpoint_{vid}_{port}', [
                'ros2', 'run', 'ros_tcp_endpoint', 'default_server_endpoint', '--ros-args',
                '-p', 'ROS_IP:=0.0.0.0', '-p', f'ROS_TCP_PORT:={port}'])
            self.viewers[vid] = {'port': port, 'endpoint': ep, 'last_ping': time.time(), 'addr': addr}
            log(f'viewer {vid} from {addr} -> endpoint port {port}')

        # wait (outside the lock) until the endpoint is listening, max 10 s
        t0 = time.time()
        while time.time() - t0 < 10:
            if not ep.running():
                break
            try:
                with open(ep.log_path, 'r', errors='replace') as f:
                    if 'Starting server' in f.read():
                        break
            except OSError:
                pass
            time.sleep(0.2)
        if not ep.running():
            self.stop(vid)
            raise RuntimeError('the endpoint failed to start: ' + ep.last_line())
        return {'ok': True, 'viewer': vid, 'port': port, 'mode': self.mode_desc}

    def ping(self, vid):
        with self.lock:
            v = self.viewers.get(vid)
            if v is None:
                return False
            v['last_ping'] = time.time()
            return True

    def stop(self, vid):
        with self.lock:
            v = self.viewers.pop(vid, None)
            if v:
                v['endpoint'].stop()
                log(f'viewer {vid} left')
            if not self.viewers and self.programs:
                log('no viewers left - stopping the mode programs')
                self._stop_mode()

    def reap(self):
        """Viewers that stopped pinging (crashed / network gone)."""
        while True:
            time.sleep(2)
            now = time.time()
            with self.lock:
                dead = [vid for vid, v in self.viewers.items()
                        if now - v['last_ping'] > self.a.viewer_timeout]
            for vid in dead:
                log(f'viewer {vid} timed out')
                self.stop(vid)

    def status(self, vid=None):
        with self.lock:
            out = {
                'ok': True,
                'host': socket.gethostname(),
                'distro': os.environ.get('ROS_DISTRO', ''),
                'domain': os.environ.get('ROS_DOMAIN_ID', '0'),
                'mode': self.mode_desc,
                'viewers': len(self.viewers),
                'programs': [p.status() for p in self.programs],
            }
            v = self.viewers.get(vid) if vid else None
            if v:
                out['programs'] = [v['endpoint'].status()] + out['programs']
            return out

    def test_pi(self, ip):
        ip = self._ip(ip)
        try:
            with socket.create_connection((ip, self.a.telemetry_port), timeout=1.5):
                return True
        except OSError:
            return False

    def shutdown(self):
        with self.lock:
            for v in self.viewers.values():
                v['endpoint'].stop()
            self.viewers.clear()
            self._stop_mode()


# --------------------------------------------------------------------------- HTTP
def make_handler(host):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):   # keep the terminal quiet (pings every 2 s)
            pass

        def _send(self, code, obj):
            body = json.dumps(obj).encode()
            try:                                                            # <<< CHANGED
                self.send_response(code)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):                 # <<< CHANGED: the viewer
                pass                                                        # <<< CHANGED: already hung up

        def _body(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            if n > 65536:
                raise ValueError('request too large')
            return json.loads(self.rfile.read(n) or b'{}')

        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            try:
                if u.path == '/status':
                    self._send(200, host.status((q.get('viewer') or [None])[0]))
                elif u.path == '/test':
                    self._send(200, {'ok': host.test_pi((q.get('ip') or [''])[0])})
                else:
                    self._send(404, {'ok': False, 'error': 'unknown path'})
            except Exception as e:
                self._send(400, {'ok': False, 'error': str(e)})

        def do_POST(self):
            u = urlparse(self.path)
            try:
                req = self._body()
                if u.path == '/start':
                    self._send(200, host.start(req, self.client_address[0]))
                elif u.path == '/ping':
                    self._send(200, {'ok': host.ping(str(req.get('viewer', '')))})
                elif u.path == '/stop':
                    # <<< CHANGED: answer at once, stop in the background (stopping
                    # a program can take up to 3 s; the viewer does not wait that long)
                    threading.Thread(target=host.stop, args=(str(req.get('viewer', '')),),
                                     daemon=True).start()
                    self._send(200, {'ok': True})
                else:
                    self._send(404, {'ok': False, 'error': 'unknown path'})
            except Exception as e:
                self._send(400, {'ok': False, 'error': str(e)})
    return Handler


def lan_ips():
    """IPv4 addresses only - that is what people type in the menu."""     # <<< CHANGED
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=2).stdout
        ips = []                                                            # <<< CHANGED
        for a in out.split():                                               # <<< CHANGED
            try:                                                            # <<< CHANGED
                ipaddress.IPv4Address(a)                                    # <<< CHANGED
                ips.append(a)                                               # <<< CHANGED
            except ValueError:                                              # <<< CHANGED
                pass                                                        # <<< CHANGED (skip IPv6)
        return ips                                                          # <<< CHANGED
    except Exception:
        return []


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if '--ros-args' in argv:              # tolerate "ros2 run ... --ros-args"
        argv = argv[:argv.index('--ros-args')]

    ap = argparse.ArgumentParser(description='VECTOR twin host (run on the lab PC)')
    ap.add_argument('--port', type=int, default=10100, help='HTTP port viewers talk to')
    ap.add_argument('--first-endpoint-port', type=int, default=10001,
                    help='first ros_tcp_endpoint port handed to viewers (10000 stays for a local twin)')
    ap.add_argument('--max-viewers', type=int, default=10)
    ap.add_argument('--viewer-timeout', type=float, default=10.0,
                    help='seconds without a ping before a viewer is dropped')
    ap.add_argument('--telemetry-port', type=int, default=9000)
    ap.add_argument('--namespace-format', default='/car_{:02d}')
    ap.add_argument('--wheel-ws', default='~/Desktop/adham_ws')
    ap.add_argument('--wheel-cmd', default='ros2 launch logitech_wheel_pkg car_control.launch.py')
    args = ap.parse_args(argv)

    host = TwinHost(args)
    threading.Thread(target=host.reap, daemon=True).start()
    server = ThreadingHTTPServer(('0.0.0.0', args.port), make_handler(host))

    def on_signal(*_):
        log('shutting down...')
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    log(f"VECTOR twin host ready - ROS 2 {os.environ.get('ROS_DISTRO', '?')}, "
        f"domain {os.environ.get('ROS_DOMAIN_ID', '0')}")
    for ip in lan_ips():
        log(f'  viewers enter this lab PC IP in the menu: {ip}   (HTTP port {args.port})')

    try:
        server.serve_forever()
    finally:
        host.shutdown()
        server.server_close()
        log('bye')


if __name__ == '__main__':
    main()
