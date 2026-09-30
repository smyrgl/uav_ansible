#!/usr/bin/env python3
"""Execute a script through the already-running official Blender MCP extension."""
import json
import socket
import sys
from pathlib import Path

def execute(code, timeout=180):
    with socket.create_connection(('127.0.0.1', 9876), timeout=10) as client:
        client.settimeout(timeout)
        client.sendall((json.dumps({'type': 'execute', 'code': code, 'strict_json': True}) + '\0').encode())
        data = bytearray()
        while b'\0' not in data:
            chunk = client.recv(1024 * 1024)
            if not chunk:
                raise RuntimeError('Blender closed the connection before returning a result')
            data.extend(chunk)
    response = json.loads(data.split(b'\0', 1)[0])
    if response.get('status') != 'ok':
        raise RuntimeError(json.dumps(response, indent=2))
    return response

if __name__ == '__main__':
    path = Path(sys.argv[1]).resolve()
    code = path.read_text()
    wrapper = f"scope = {{'__name__': '__main__', '__file__': {str(path)!r}}}\nexec(compile({code!r}, {str(path)!r}, 'exec'), scope)\nresult = scope.get('result', {{}})"
    print(json.dumps(execute(wrapper), indent=2, ensure_ascii=False))
