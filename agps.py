import socket

sock = socket.socket()
address = "agps.u-blox.com"
port = 46434
print "Connecting to u-blox"
sock.connect((address, port))
print "Connection established"

print "Sending the request"
sock.send("cmd=full;user=korovkin@gmail.com;token=4HWt1EvhQUKJ2InFyaaZDw;lat=30.0;lon=30.0;pacc=10000;")
print "Sending the request - done"

data = ""
buffer = True;
while buffer:
    print(".")
    buffer = sock.recv(1024)
    if buffer:
        data += buffer
print("\n")
print(data)
