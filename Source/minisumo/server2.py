#!/usr/bin/env python3
import sys, os, socket
import json
import shutil
from socketserver import ThreadingMixIn
from http.server import SimpleHTTPRequestHandler, HTTPServer

HOST = socket.gethostname()


class MyHttpRequestHandler(SimpleHTTPRequestHandler):

    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        
    def end_headers(self):
        self.send_my_headers()
        SimpleHTTPRequestHandler.end_headers(self)

    def send_my_headers(self):
        self.send_header("COMMAND", "1")

    def do_GET(self):
        SimpleHTTPRequestHandler.do_GET(self)
        with open("commands.html","w") as fp:
            fp.write("NOTHING")
        print("After GET")
    
    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        post_data = self.rfile.read(content_length) # <--- Gets the data itself
        command = json.loads(post_data.decode('utf-8'))
        self._set_response()
        self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))
        with open("commands.html","w") as fp:
            cmd = command["comando"]
            # Copy binary file
            if "UPDATE_FW" in cmd:
                shutil.copy(".pio/build/esp32doit-devkit-v1/firmware.bin", "firmware.bin")
            fp.write(command["comando"])


class ThreadingSimpleServer(ThreadingMixIn, HTTPServer):
    pass

PORT = 8000
CWD = os.getcwd()

server = ThreadingSimpleServer(('0.0.0.0', PORT), MyHttpRequestHandler)
print("Serving HTTP traffic from", CWD, "on", HOST, "using port", PORT)
try:
    while 1:
        sys.stdout.flush()
        server.handle_request()
except KeyboardInterrupt:
    print("\nShutting down server per users request.")