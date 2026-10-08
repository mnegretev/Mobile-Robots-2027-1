import  http.server
import socketserver
import os
from ament_index_python.packages import get_package_share_directory


def main():
    package_share = get_package_share_directory('car_web_control')
    web_dir = os.path.join(package_share, 'web')
    os.chdir(web_dir)
    PORT = 8000
    Handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", PORT), Handler) as httpd: 
        print(f'Car web interface running at: ')
        print(f'http://localhost:{port}')
        http.serve_forever()


if __name__ == "__main__":
    main()