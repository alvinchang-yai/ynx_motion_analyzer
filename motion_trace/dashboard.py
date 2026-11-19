"""Local web dashboard for the motion analyzer: ros2 run ynx_motion_analyzer dashboard

Serves a single page (dashboard/ in this package's share directory) plus a small JSON
API over the recordings in experiment/. Analysis only - nothing here talks to the
robot. Python standard library only; binds to localhost by default.
"""
import argparse
import json
import mimetypes
import os
import threading
import traceback
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from motion_trace.analysis import analyze_bag, list_bags
from motion_trace.hardware_profile import PACKAGE_NAME
from motion_trace.record_motion import experiment_dir


def static_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory(PACKAGE_NAME), 'dashboard')
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'dashboard')


class _Cache:
    # Analysis results keyed by (bag, its newest file mtime, query) - recomputed only
    # when the recording changes.
    def __init__(self):
        self._lock, self._data = threading.Lock(), {}

    def get(self, key, compute):
        with self._lock:
            if key in self._data:
                return self._data[key]
        value = compute()
        with self._lock:
            self._data[key] = value
        return value


def make_handler(exp_dir, static_root):
    cache = _Cache()

    class Handler(BaseHTTPRequestHandler):
        server_version = 'MotionAnalyzerDashboard/1'

        def log_message(self, fmt, *args):  # quieter than the default per-request log
            if not self.path.startswith('/static/'):
                super().log_message(fmt, *args)

        def _send(self, status, body, content_type):
            data = body if isinstance(body, bytes) else body.encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _json(self, obj, status=HTTPStatus.OK):
            body = json.dumps(_finite(obj), allow_nan=False, default=_json_default)
            self._send(status, body, 'application/json')

        def _error(self, status, message):
            self._json({'error': message}, status)

        def do_GET(self):
            url = urlparse(self.path)
            query = {k: v[-1] for k, v in parse_qs(url.query).items()}
            try:
                if url.path in ('/', '/index.html'):
                    return self._static('index.html')
                if url.path.startswith('/static/'):
                    return self._static(url.path[len('/static/'):])
                if url.path == '/api/bags':
                    return self._json({'experiment_dir': exp_dir, 'bags': list_bags(exp_dir)})
                if url.path == '/api/analysis':
                    return self._analysis(query)
                return self._error(HTTPStatus.NOT_FOUND, f'No such page: {url.path}')
            except SystemExit as e:  # profile/topic problems are reported via SystemExit
                return self._error(HTTPStatus.UNPROCESSABLE_ENTITY, str(e))
            except Exception as e:
                traceback.print_exc()
                return self._error(HTTPStatus.INTERNAL_SERVER_ERROR, f'{type(e).__name__}: {e}')

        def _static(self, rel):
            root = os.path.realpath(static_root)
            path = os.path.realpath(os.path.join(root, rel))
            if not path.startswith(root + os.sep) or not os.path.isfile(path):
                return self._error(HTTPStatus.NOT_FOUND, f'No such file: {rel}')
            ctype = mimetypes.guess_type(path)[0] or 'application/octet-stream'
            with open(path, 'rb') as f:
                self._send(HTTPStatus.OK, f.read(), ctype)

        def _analysis(self, query):
            name = query.get('bag', '')
            # Only recordings that the listing shows - never an arbitrary path.
            names = {b['name'] for b in list_bags(exp_dir) if b['readable']}
            if name not in names:
                return self._error(HTTPStatus.NOT_FOUND, f"No readable recording named '{name}' in {exp_dir}.")
            path = os.path.join(exp_dir, name)
            mtime = max(os.path.getmtime(os.path.join(path, f)) for f in os.listdir(path))
            result = cache.get((name, mtime), lambda: analyze_bag(path))
            return self._json(result)

    return Handler


def _finite(o):
    # JSON has no NaN/Infinity (browsers reject them): send null instead.
    if isinstance(o, float):
        return o if o == o and o not in (float('inf'), float('-inf')) else None
    if isinstance(o, dict):
        return {k: _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_finite(v) for v in o]
    return o


def _json_default(o):
    try:
        return float(o)
    except (TypeError, ValueError):
        return str(o)


def main():
    parser = argparse.ArgumentParser(description='Local web dashboard for recordings in experiment/.')
    parser.add_argument('--port', type=int, default=8050)
    parser.add_argument('--host', default='127.0.0.1',
                        help='Address to bind (default 127.0.0.1: this PC only). Use 0.0.0.0 to share on the LAN.')
    args, _ = parser.parse_known_args()

    exp_dir, root = experiment_dir(), static_dir()
    mimetypes.add_type('text/javascript', '.js')
    server = ThreadingHTTPServer((args.host, args.port), make_handler(exp_dir, root))
    print(f'Motion analyzer dashboard: http://{"localhost" if args.host == "127.0.0.1" else args.host}:{args.port}')
    print(f'  recordings: {exp_dir}')
    print('  Ctrl+C to stop.')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
