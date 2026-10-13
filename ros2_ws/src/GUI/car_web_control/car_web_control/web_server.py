#!/usr/bin/env python3

import os
from functools import partial
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler

from ament_index_python.packages import get_package_share_directory


def main(args=None):

    package_share = get_package_share_directory(
        'car_web_control'
    )

    web_dir = os.path.join(package_share, 'web')

    port = 7000

    handler = partial(
        SimpleHTTPRequestHandler,
        directory=web_dir
    )

    server = ThreadingHTTPServer(
        ('127.0.0.1', port),
        handler
    )

    print(
        f'Car Web GUI running at http://localhost:{port}',
        flush=True
    )

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()