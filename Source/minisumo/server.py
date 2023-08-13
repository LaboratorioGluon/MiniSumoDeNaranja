import http.server
import socketserver
import json

class MyHttpRequestHandler(http.server.SimpleHTTPRequestHandler):

    def _set_response(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

        
    def end_headers(self):
        self.send_my_headers()
        http.server.SimpleHTTPRequestHandler.end_headers(self)

    def send_my_headers(self):
        self.send_header("COMMAND", "1")

    def do_GET(self):
        http.server.SimpleHTTPRequestHandler.do_GET(self)
        with open("commands.html","w") as fp:
            fp.write("NOTHING")
        print("After GET")
    
    def do_POST(self):
        content_length = int(self.headers['Content-Length']) # <--- Gets the size of data
        post_data = self.rfile.read(content_length) # <--- Gets the data itself
        print(post_data.decode('utf-8'))
        command = json.loads(post_data.decode('utf-8'))
        print(command)
        self._set_response()
        self.wfile.write("POST request for {}".format(self.path).encode('utf-8'))
        with open("commands.html","w") as fp:
            fp.write(command["comando"])
        #return http.server.SimpleHTTPRequestHandler.do_POST(self)

handler_object = MyHttpRequestHandler
my_server = socketserver.TCPServer(("", 8000), handler_object)
my_server.serve_forever()