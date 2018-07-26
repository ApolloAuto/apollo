import socket
import threading


def recv_udp(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = ("", port)
    s.bind(address)
    i = 0
    while True:
        data, client = s.recvfrom(1500)
        print("index: %d, client: %s, date len: %s" % (i, client, len(data)))
        i += 1
    s.close()

if __name__ == '__main__': 
    t_pos = threading.Thread(target = recv_udp, args = (8308,), name = 'pos')
    t_pos.start()

    t_data = threading.Thread(target = recv_udp, args = (2368,), name = 'data')
    t_data.start()

    t_pos.join()
    t_data.join()
