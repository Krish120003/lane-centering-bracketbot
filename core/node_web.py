from http.server import BaseHTTPRequestHandler, HTTPServer

class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # Serve the web interface
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            # Send the HTML content
            html_content = '''<!DOCTYPE html>
<html>
<head>
    <title>Robot Controller</title>
    <!-- Include MQTT.js from CDN -->
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <script src="https://unpkg.com/mqtt/dist/mqtt.min.js"></script>
    <style>
        .control-pad {
            display: grid;
            grid-template-columns: repeat(3, 80px);
            gap: 10px;
            margin: 20px;
        }
        .control-button {
            width: 80px;
            height: 80px;
            font-size: 24px;
            /* Disable text selection/highlighting on mobile */
            -webkit-touch-callout: none;
            -webkit-user-select: none;
            user-select: none;
            /* Prevent default touch behaviors */
            touch-action: manipulation;
        }
        .speech-box input {
            /* Make input more touch-friendly */
            font-size: 16px;
            padding: 8px;
            width: 200px;
        }
        .speech-box button {
            /* Make speech button more touch-friendly */
            font-size: 16px;
            padding: 8px 16px;
            margin-left: 8px;
        }
        #status {
            margin: 20px;
            color: red;
            /* Make status text more readable */
            font-size: 18px;
        }
    </style>
</head>
<body>
    <div id="status">Disconnected</div>
    
    <div class="control-pad">
        <button class="control-button" ontouchstart="sendCommand('forward_left')" ontouchend="sendCommand('stop')"> </button>
        <button class="control-button" ontouchstart="sendCommand('forward')" ontouchend="sendCommand('stop')">^</button>
        <button class="control-button" ontouchstart="sendCommand('forward_right')" ontouchend="sendCommand('stop')"> </button>
        <button class="control-button" ontouchstart="sendCommand('left')" ontouchend="sendCommand('stop')"><</button>
        <button class="control-button" ontouchstart="sendCommand('stop')" ontouchend="sendCommand('stop')">+</button>
        <button class="control-button" ontouchstart="sendCommand('right')" ontouchend="sendCommand('stop')">></button>
        <button class="control-button" ontouchstart="sendCommand('back_left')" ontouchend="sendCommand('stop')"> </button>
        <button class="control-button" ontouchstart="sendCommand('back')" ontouchend="sendCommand('stop')">v</button>
        <button class="control-button" ontouchstart="sendCommand('back_right')" ontouchend="sendCommand('stop')"> </button>
    </div>

    <div class="speech-box">
        <input type="text" id="speechText" placeholder="Enter text to speak">
        <button onclick="sendSpeech()">Speak</button>
    </div>

    <script>
        // Extract username from URL or use hostname
        const hostname = window.location.hostname;  // e.g., "beige-desktop.local" or "192.168.1.100"
        const username = hostname.split('-')[0];    // e.g., "beige" or use full hostname if no hyphen
        
        // MQTT client setup - use the extracted username or full hostname
        const mqttHost = username.includes('.') ? hostname : `${username}-desktop.local`;
        const client = mqtt.connect(`ws://${mqttHost}:9001`);
        
        client.on('connect', function () {
            document.getElementById('status').style.color = 'green';
            document.getElementById('status').innerHTML = 'Connected';
        });

        client.on('error', function (error) {
            document.getElementById('status').style.color = 'red';
            document.getElementById('status').innerHTML = 'Connection failed: ' + error;
        });

        client.on('message', function (topic, message) {
            console.log("Message received:", message.toString());
        });

        // Add state tracking for keys
        const keyState = {
            w: false,
            a: false,
            s: false,
            d: false
        };

        // Update keyboard controls
        document.addEventListener('keydown', function(event) {
            const key = event.key.toLowerCase();
            if (keyState.hasOwnProperty(key)) {
                keyState[key] = true;
                updateCommand();
            } else if (event.key === ' ') {  // spacebar
                // Reset everything
                Object.keys(keyState).forEach(k => keyState[k] = false);
                sendCommand('stop');
            }
        });

        document.addEventListener('keyup', function(event) {
            const key = event.key.toLowerCase();
            if (keyState.hasOwnProperty(key)) {
                keyState[key] = false;
                updateCommand();
            }
        });

        function updateCommand() {
            if (keyState['w'] && keyState['a']) {
                sendCommand('forward_left');
            } else if (keyState['w'] && keyState['d']) {
                sendCommand('forward_right');
            } else if (keyState['s'] && keyState['a']) {
                sendCommand('back_left');
            } else if (keyState['s'] && keyState['d']) {
                sendCommand('back_right');
            } else if (keyState['w']) {
                sendCommand('forward');
            } else if (keyState['s']) {
                sendCommand('back');
            } else if (keyState['a']) {
                sendCommand('left');
            } else if (keyState['d']) {
                sendCommand('right');
            } else {
                sendCommand('stop');
            }
        }

        // Sending messages
        function sendCommand(command) {
            client.publish("robot/drive", command);
        }

        function sendSpeech() {
            const text = document.getElementById('speechText').value;
            client.publish("robot/speak", text);
            document.getElementById('speechText').value = '';
        }
    </script>
</body>
</html>
'''
            self.wfile.write(html_content.encode('utf-8'))
        else:
            self.send_error(404, 'File Not Found: %s' % self.path)
                
def run(server_class=HTTPServer, handler_class=MyHandler, port=8080):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print(f"Serving HTTP on port {port}...")
    httpd.serve_forever()

if __name__ == '__main__':
    run()
