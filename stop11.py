import socket


ROBOT_IP = '192.168.1.55'#'192.168.43.120' #'192.168.43.68' ##'192.168.0.148' #"127.0.0.1"
ROBOT_PORT = 3000 #3001
LEFT_TRACK_SPEED = 0#-100
RIGHT_TRACK_SPEED = 0


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(bytes(f"{LEFT_TRACK_SPEED};{RIGHT_TRACK_SPEED}", "utf-8"),
                (ROBOT_IP, ROBOT_PORT))


if __name__ == '__main__':
    main()
   