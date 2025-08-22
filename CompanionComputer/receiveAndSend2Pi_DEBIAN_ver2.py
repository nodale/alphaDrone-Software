#!/usr/bin/env python3

import socket
import redis
import time
import json

REDIS_HOST = 'localhost'            
REDIS_PORT = 6379                   
REDIS_DB = 0                        
REDIS_PASSWORD = 'yes' 
REDIS_KEY = 'translation'          

TARGET_IP = 'localhost'          
TARGET_PORT = 65432                

def get_data_from_redis(psub):
    for message in psub.listen():
        if message['type'] == 'message':
            yield message['data']

def send_data_over_socket(data, s):
    try:
        if isinstance(data, str):
            data = data.encode('utf-8')
        s.sendall(data + b'\n') 
#        print(f"[✓] Sent {len(data)} bytes")
    except Exception as e:
        print(f"[✗] Socket error: {e}")

def main():

    counter = 0
    try:
        r = redis.Redis(
            host=REDIS_HOST,
            port=REDIS_PORT,
            db=REDIS_DB,
            password=REDIS_PASSWORD
        )
        if r.ping():
            print("[✓] Connected to Redis")
    except Exception as e:
        print(f"[✗] Redis connection failed: {e}")
        return
    psub = r.pubsub()
    psub.subscribe('channel_pose')

    redis_gen = get_data_from_redis(psub)
    ip, port = TARGET_IP, TARGET_PORT
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, port))

        for data in redis_gen:   
            if data:
                send_data_over_socket(data, s)
                counter = counter + 1
                print(f'counter = {counter}')
#                time.sleep(0.1)

if __name__ == "__main__":
    main()


