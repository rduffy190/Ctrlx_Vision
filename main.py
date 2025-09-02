#!/usr/bin/env python3

# SPDX-FileCopyrightText: Bosch Rexroth AG
#
# SPDX-License-Identifier: MIT

import os
import time
from hdv.object_detection2d import Result, Instance, OrientedBoundingBox
from http.server import HTTPServer
from appdata.app_data_control import AppDataControl
from api_helper.ctrlx_api import CtrlxDlAPi
from app_server.server import Server, UnixSocketHttpServer
import ctrlxdatalayer
from vision.vision import run_vision
from vision.error_codes import ErrorCodes
import signal
from threading import Thread
__close_app = False

def handler(signum, frame):
    """handler"""
    global __close_app
    __close_app = True


def main():
    """main
    """
    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGABRT, handler)

    app_data = AppDataControl()
    app_data.load()
    app_server = get_app_server()
    t= Thread(target= run_app_server,args=(app_server,))
    t.start()

    camera_node = app_data.get_appdata()['camera_node']
    infer_node = app_data.get_appdata()['inference_node']
    api = CtrlxDlAPi()
    system = ctrlxdatalayer.system.System('')
    system.start(False)
    api.start_sys(system) 
    api.create_end_points()
    while (not __close_app) and api.is_connected(): 
        if api.get_request(): 
            api.write_busy(True)
            try_count = 0
            while try_count < 10: 
                success = run_vision(api,camera_node,infer_node) 
                if success[0]:
                    break
                try_count += 1

            api.write_busy(False)
            if not success[0]: 
                api.write_error(True)
                api.write_error_code(success[1])
            else:
                api.write_done(True)
            while api.get_request(): 
                time.sleep(.05)
            api.write_done(False)
            api.write_error(False)
            api.write_error_code(ErrorCodes.NO_ERROR)
        else: 
            time.sleep(0.05)
    system.stop(True)
    app_server.shutdown()
    t.join()

def get_app_server(): 
    # Start webserver to get load/save rest requests
    # Use unix sockets if app is running as snap
    if 'SNAP' in os.environ:
        webserver = create_webserver_unixsock()
    else:
        webserver = create_webserver_tcp()
def run_app_server(webserver):
    webserver.serve_forever()
    

def create_webserver_tcp():
    """create_webserver_tcp
    """
    server_port = 1234
    hostname = 'localhost'

    webserver = HTTPServer(('', server_port), Server)
    return webserver


def create_webserver_unixsock():
    """create_webserver_unixsock
    """
    sock_dir = os.getenv('SNAP_DATA') + '/package-run/sdk-py-appdata/'
    sock_file = sock_dir + 'web.sock'
    if not os.path.exists(sock_dir):
        os.makedirs(sock_dir)
    try:
        os.unlink(sock_file)
    except OSError:
        pass

    webserver = UnixSocketHttpServer(sock_file, Server)
    return webserver


if __name__ == '__main__':
    main()
