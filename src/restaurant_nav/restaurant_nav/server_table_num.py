import socket

def send_table_num(table_num):
    # Configuration
    PUBLISH_HOST = '127.0.0.1'  # Replace with the target IP
    PUBLISH_PORT = 12345

    # Create a socket for sending
    publisher_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    publisher_socket.connect((PUBLISH_HOST, PUBLISH_PORT))

    # Data to send
    data_to_send = f"{table_num}"
    publisher_socket.sendall(data_to_send.encode())
    print(f"Sent data: {data_to_send}")
    publisher_socket.close()


send_table_num(12)



