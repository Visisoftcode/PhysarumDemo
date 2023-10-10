#!/usr/bin/python3

import http.server
import socketserver

class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
	def end_headers(self):
		self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, private')
		super().end_headers()

host = 'localhost'
port = 8080

with socketserver.TCPServer((host, port), NoCacheHandler) as httpd:
	print(f"Serving at {host}:{port}")
	httpd.serve_forever()
