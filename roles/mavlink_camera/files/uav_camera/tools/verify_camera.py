#!/usr/bin/env python3
"""Bounded camera-only MAVLink acceptance probe; --capture writes test media."""
import argparse
import json
import socket
import time
from pymavlink.dialects.v20 import common as mav

p=argparse.ArgumentParser()
p.add_argument('--host', default='192.168.144.1')
p.add_argument('--port', type=int, default=5760)
p.add_argument('--capture', action='store_true')
a=p.parse_args()
sock=socket.create_connection((a.host,a.port),timeout=5)
sock.settimeout(0.5)
parser=mav.MAVLink(None,srcSystem=253,srcComponent=191)
parser.robust_parsing=True
messages=[]

def receive(timeout, predicate):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        try:
            raw=sock.recv(65536)
            if not raw: raise RuntimeError('router disconnected')
        except socket.timeout: continue
        batch = [m for m in (parser.parse_buffer(raw) or [])
                 if m.get_srcSystem()==1 and m.get_srcComponent()==100]
        messages.extend(batch)
        for msg in batch:
            if predicate(msg): return msg
    raise TimeoutError('expected camera response not received')

def command(command_id,*params):
    packet=mav.MAVLink_command_long_message(1,100,command_id,0,*(list(params)+[0.]*(7-len(params))))
    sock.sendall(packet.pack(parser))
    parser.seq=(parser.seq+1)&255
    ack=receive(10,lambda m:m.get_type()=='COMMAND_ACK' and m.command==command_id and m.result!=mav.MAV_RESULT_IN_PROGRESS)
    if ack.result != mav.MAV_RESULT_ACCEPTED: raise RuntimeError(f'command {command_id} result {ack.result}')
    return ack

receive(8,lambda m:m.get_type()=='HEARTBEAT')
for msgid in (mav.MAVLINK_MSG_ID_CAMERA_INFORMATION,mav.MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION,mav.MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS):
    command(mav.MAV_CMD_REQUEST_MESSAGE,msgid)
# Drain the separately emitted requested payloads without assuming ACK ordering.
receive(3,lambda m:m.get_type()=='CAMERA_CAPTURE_STATUS')
info=next(m for m in reversed(messages) if m.get_type()=='VIDEO_STREAM_INFORMATION')
status=next(m for m in reversed(messages) if m.get_type()=='CAMERA_CAPTURE_STATUS')
result={'stream':info.to_dict(),'initial_capture_status':status.to_dict()}
if a.capture:
    if status.video_status: raise RuntimeError('recording already active; refusing to interrupt it')
    started=False
    try:
        command(mav.MAV_CMD_VIDEO_START_CAPTURE,0,1)
        started=True
        start=time.monotonic()
        command(mav.MAV_CMD_IMAGE_START_CAPTURE,0,0,1,int(time.time())&0xFFFFFF)
        event=next((m for m in reversed(messages) if m.get_type()=='CAMERA_IMAGE_CAPTURED'),None)
        if event is None: event=receive(4,lambda m:m.get_type()=='CAMERA_IMAGE_CAPTURED')
        if event.capture_result != 1: raise RuntimeError('image capture failed')
        result['photo_event']=event.to_dict()
        while time.monotonic()-start<5:
            receive(3,lambda m:m.get_type()=='HEARTBEAT')
    finally:
        if started: command(mav.MAV_CMD_VIDEO_STOP_CAPTURE,0)
    result['capture_verified']=True
print(json.dumps(result,indent=2))
sock.close()
