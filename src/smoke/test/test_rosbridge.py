"""Check that a real rosbridge server accepts a WebSocket connection."""

import asyncio
from pathlib import Path
import socket
import subprocess

from ament_index_python.packages import get_package_prefix
from tornado.httpclient import HTTPClientError
from tornado.websocket import websocket_connect


def test_rosbridge_websocket_connection(tmp_path):
    # Use a spare local port so the test can coexist with a development server.
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        port = listener.getsockname()[1]

    executable = (
        Path(get_package_prefix("rosbridge_server"))
        / "lib/rosbridge_server/rosbridge_websocket"
    )
    log_path = tmp_path / "rosbridge.log"
    with log_path.open("w") as log:
        server = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-p",
                f"port:={port}",
                "-p",
                "address:=127.0.0.1",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
        )

        async def connect():
            loop = asyncio.get_running_loop()
            deadline = loop.time() + 15
            last_error = None
            while loop.time() < deadline:
                assert server.poll() is None, (
                    f"rosbridge exited before connecting:\n{log_path.read_text()}"
                )
                try:
                    connection = await websocket_connect(
                        f"ws://127.0.0.1:{port}",
                        connect_timeout=min(1, deadline - loop.time()),
                    )
                except (OSError, HTTPClientError, asyncio.TimeoutError) as error:
                    last_error = error
                    await asyncio.sleep(0.1)
                else:
                    # websocket_connect completes only after the WebSocket handshake.
                    connection.close()
                    return
            raise AssertionError(
                f"No rosbridge WebSocket connection within 15 seconds: {last_error}\n"
                f"{log_path.read_text()}"
            )

        try:
            asyncio.run(connect())
        finally:
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait(timeout=5)
