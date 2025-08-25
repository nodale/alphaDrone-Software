from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2
import time
import math
import msgpack

import socket
import numpy as np

filename = 'pos.txt'
with open(filename, 'w') as f:
    f.write('x1\ty1\tz1\tx2\ty2\tz2\tx_sp\ty_sp\tz_sp\tt\n')

filename_q = 'q.txt'
with open(filename_q, 'w') as fq:
    fq.write('x1\ty1\tz1\tx2\ty2\tz2\tt\n')

filename_ang = 'ang.txt'
with open(filename_ang, 'w') as fa:
    fa.write('x1\ty1\tz1\tx2\ty2\tz2\tt\n')

filename_v = 'vel.txt'
with open(filename_v, 'w') as fv:
    fv.write('x1\ty1\tz1\tx2\ty2\tz2\tt\n')

time_boot = time.time()

HOST = 'localhost'
PORT = 65432
scaler = 1000
x_p, y_p, z_p, qw_p, qx_p, qy_p, qz_p = 0, 0, 0, 0.5, 0, 0, 0
x_c, y_c, z_c, qw, qx, qy, qz = 0, 0, 0, 0, 0, 0, 0
x_os , y_os, z_os = 0, 0, 0
vx_p, vy_p, vz_p = 0, 0, 0
x_p2, y_p2, z_p2 = 0, 0, 0
x_sp, y_sp, z_sp = 0, 0, 0
COV_LIST = [0.002]*21
dt = 1
timeStep_c = 1
timeStep_p = 0
counter = 0

def append_row(x1, y1, z1, x2, y2, z2, t, name):
    row = np.array([[x1, y1, z1, x2, y2, z2, t]])
    with open(name, 'a') as f:
        np.savetxt(f, row, fmt='%.6f', delimiter='\t')

def append_row_pos(x1, y1, z1, x2, y2, z2, x_sp, y_sp, z_sp, t, name):
    row = np.array([[x1, y1, z1, x2, y2, z2, x_sp, y_sp, z_sp, t]])
    with open(name, 'a') as f:
        np.savetxt(f, row, fmt='%.6f', delimiter='\t')

def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Sent arm command")

def send_position_target(x, y, z): 
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x, y, z,  #position
        0, 0, 0,  #velocity
        0, 0, 0,  #acceleration
        0, 0  #yaw yaw_rate
    )

def euler_rates_from_quats(qw1, qx1, qy1, qz1,
                           qw2, qx2, qy2, qz2, dt):
    q1 = np.array([qw1, qx1, qy1, qz1])
    q2 = np.array([qw2, qx2, qy2, qz2])
    
    q_conj = np.array([q1[0], -q1[1], -q1[2], -q1[3]])
    
    w1, x1, y1, z1 = q_conj
    w2, x2, y2, z2 = q2
    q_rel = np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])
    q_rel /= np.linalg.norm(q_rel)
    
    angle = 2*np.arccos(np.clip(q_rel[0], -1, 1))
    if angle < 1e-8:
        return np.zeros(3)
    axis = q_rel[1:] / np.sin(angle/2)
    omega = (angle/dt) * axis
    
    w, x, y, z = q1
    roll  = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1, 1))
    
    phi, theta = roll, pitch
    T = np.array([
        [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi),              -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ])
    return T @ omega  

def quaternion_to_euler(qw, qx, qy, qz):
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx**2 + qy**2)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))  # clamp
    pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy**2 + qz**2)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def lpf(new_value, prev_value, alpha=0.6):
    return alpha * new_value + (1 - alpha) * prev_value
                    
def get_local_position(master):
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    return msg

master = mavutil.mavlink_connection('/dev/ttyACM0', baudrate=57600)

master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
print("Trying to connect")

master.wait_heartbeat(timeout=1)
print("Connected to system")

master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    100, 
    1   
    )
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION,
    100, 
    1   
    )

master.set_mode("STABILIZED")
#arm()
time.sleep(1)

initLoc = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)

x_os = initLoc.x
y_os = initLoc.y
z_os = initLoc.z

x_sp = x_os
y_sp = y_os
z_sp = z_os - 0.8

omega = 0.5
R = 0.6


print(f"init pos=({initLoc.x:.3f},{initLoc.y:.3f},{initLoc.z})")

time.sleep(0.5)

q_lpf_param = 1.0
v_lpf_param = 0.1
#v_lpf_param = 0.06
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Listening on {HOST}:{PORT}")

    conn, addr = s.accept()
    print(f"Connected")
    

    with conn:
        unpacker = msgpack.Unpacker(raw=False) 
        
        chunk = conn.recv(2048)

        unpacker.feed(chunk)

        for msg in unpacker:
            if isinstance(msg, list) and len(msg) > 0 and isinstance(msg[0], dict):
                data = msg[0]
                translation, translation_flag = data['translation']
                x_p = translation[0] / scaler
                y_p = translation[1] / scaler
                z_p = translation[2] /scaler
        time_p = time.time()
        time.sleep(0.2)

        t0 = time.time()

        while True:
            time_c = time.time()
            dt = time_c - time_p
            chunk = conn.recv(2048)


            if not chunk: 
                break

            unpacker.feed(chunk)
            for msg in unpacker:
                if isinstance(msg, list) and len(msg) > 0 and isinstance(msg[0], dict):
                    data = msg[0]
                    translation, translation_flag = data['translation']
                    quanternion, quanternion_flag = data['quanternion']
                    velocity = data['velocity']
                    
                    vx = velocity[0]
                    vy = -velocity[1]
                    vz = -velocity[2]

                    qx = quanternion[0]
                    qy = quanternion[1]
                    qz = quanternion[2]

                    x = (translation[0]/scaler) - x_p + x_os
                    y = (translation[1]/scaler) - y_p + y_os
                    z = (-translation[2]/scaler) - z_p + z_os


                    x_fil = lpf(x, x_p2, 1)
                    y_fil = lpf(y, y_p2, 1)
                    z_fil = lpf(z, z_p2, 1)

                    vx_fil = lpf(vx, vx_p, v_lpf_param)
                    vy_fil = lpf(vy, vy_p, v_lpf_param)
                    vz_fil = lpf(vz, vz_p, v_lpf_param)
                    
                    qw_fil = lpf(quanternion[3], qw_p, q_lpf_param)
                    qx_fil = lpf(qx, qx_p, q_lpf_param)
                    qy_fil = lpf(qy, qy_p, q_lpf_param)
                    qz_fil = lpf(qz, qz_p, q_lpf_param)

                    PRYrates = euler_rates_from_quats(qw_fil, qx_fil, qy_fil, qz_fil, qw_p, qx_p, qy_p, qz_p, dt)

                    vodom = mavlink2.MAVLink_odometry_message(
                        int(time_c * 1e6),
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        mavutil.mavlink.MAV_FRAME_BODY_FRD,
                        x_fil, y_fil, z_fil,
                        [qw_fil, qx_fil, qy_fil, qz_fil],
                        vx_fil, vy_fil, vz_fil,
                        *PRYrates,
                        COV_LIST,
                        COV_LIST,
                        0,
                        0,
                        0
                    )

                    master.mav.send(vodom)
                    
                    t = time.time() - t0
                    x_sp = x_os + R * math.cos(omega * t)
                    y_sp = y_os + R * math.sin(omega * t)
                    send_position_target(x_sp, y_sp, z_sp)


                    nowLoc = get_local_position(master)

                    nowAng = master.recv_match(type='ATTITUDE', blocking=False)
                    nowQ = master.recv_match(type='ODOMETRY', blocking=False)
                    #quaternion debug
                    if nowQ:
                        append_row(nowQ.q[1], nowQ.q[2], nowQ.q[3], qx_fil, qy_fil, qz_fil, time_c - time_boot, filename_q)
                        #print(f"qw = {qw:.3f}, qx = {qx:.3f}, qy = {qy:.3f}, qz = {qz:.3f}")
                    #angle debug
                    if nowAng:
                        t_pitch, t_roll, t_yaw = quaternion_to_euler(qw_fil, qx_fil, qy_fil, qz_fil)
                        append_row(nowAng.roll, nowAng.pitch, nowAng.yaw, t_roll, t_pitch, t_yaw, time_c - time_boot, filename_ang)

                        #print(f"roll = {t_roll:.3f}, pitch = {t_pitch:.3f}, yaw = {t_yaw:.3f}")
                    #pos debug
                    if nowLoc:
                        append_row_pos(nowLoc.x, nowLoc.y, nowLoc.z, x_fil, y_fil, z_fil, x_sp, y_sp, z_sp, time_c - time_boot, filename)
                        print(f"target err=({x_sp - x:.3f}, {y_sp - y:.3f}, {z_sp - z:.3f}), est error =({nowLoc.x - x:.3f}, {nowLoc.y - y:.3f}, {nowLoc.z - z:.3f}), counter = {counter}")
                    #vel debug
                    if nowLoc:
                        append_row(nowLoc.vx, nowLoc.vy, nowLoc.vz, vx_fil, vy_fil, vz_fil, time_c - time_boot, filename_v)

                    counter += 1
                    time_p = time_c
                    #qw_p, qx_p, qy_p, qz_p = quanternion[3], quanternion[0], quanternion[1], quanternion[2]
                    #vx_p, vy_p, vz_p =  velocity[0], velocity[1], velocity[2]
                else:
                    master.mav.send(vodom)
                    send_position_target(x_sp, y_sp, z_sp)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)

print("Disarmed")
