#!/usr/bin/env python

import socketserver
import rospy
from std_msgs.msg import String


class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        while 1:
            # self.request is the TCP socket connected to the client
            self.data = self.request.recv(1024)
            self.data.strip()
            if not self.data:
                print("Out")
                break
            print("{} wrote:".format(self.client_address[0]))
            print(self.data)

            if not rospy.is_shutdown():
                rospy.loginfo(self.data)
                pub.publish(self.data)

            # just send back the same data, but upper-cased
            # self.request.send(self.data.upper())


if __name__ == "__main__":
    HOST, PORT = "192.168.1.143", 4440
    rospy.init_node('phone_text')
    pub = rospy.Publisher('phone_text', String, queue_size=10)

    # Create the server, binding to localhost on port 9999
#    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
    server = socketserver.TCPServer((HOST, PORT), MyTCPHandler)
    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    print(server.server_address)
    print("Now receiving...")
    server.serve_forever()

