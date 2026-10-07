#!/usr/bin/env python3
"""LAN portal and local helper entry points, Python 3.11+, no pip dependencies.

Root is isolated behind a private UNIX socket with peer-UID authorization. The
network process serves assets/status and forwards only typed maintenance actions.
"""
import argparse
import base64
import functools
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import signal
import socket
import socketserver
import struct
import threading
import time
from urllib.parse import urlparse
from urllib.request import urlopen

from runtime import Engine

STATIC=Path(__file__).parent/'static'
MAX_REQUEST=16384

def request_local(path, method, args=None):
    with socket.socket(socket.AF_UNIX) as s:
        s.settimeout(5); s.connect(str(path)); f=s.makefile('rwb')
        f.write((json.dumps({'method':method,'args':args or {}})+'\n').encode()); f.flush()
        data=f.readline(1048577)
        if not data or len(data)>1048576: raise RuntimeError('本地辅助器响应过长/断开')
        obj=json.loads(data)
        if not obj.get('ok'): raise RuntimeError(obj.get('error','辅助器错误'))
        return obj['result']

def dispatch(engine, method, args):
    if method=='status': return engine.snapshot()
    if method=='job':
        if set(args)!={'id'}: raise ValueError('任务查询参数错误')
        return engine.jobs.get(args['id'],{'status':'不存在'})
    if method=='report': return {'status':engine.snapshot(),'summary':list(engine.summary)}
    if method=='audit':
        ident=args.get('id','')
        if len(ident)!=32 or any(c not in '0123456789abcdef' for c in ident): raise ValueError('审计ID错误')
        for p in sorted((engine.root/'audit').glob('events-*.jsonl'),reverse=True):
            for line in p.read_text().splitlines():
                row=json.loads(line)
                if row['id']==ident: return row
        raise ValueError('审计记录不存在')
    if method=='action':
        if set(args)!={'action','params'} or not isinstance(args['params'],dict): raise ValueError('操作封装错误')
        return engine.submit(args['action'],args['params'])
    raise ValueError('未知本地接口')

class HelperHandler(socketserver.StreamRequestHandler):
    def handle(self):
        self.request.settimeout(5)
        if hasattr(socket,'SO_PEERCRED'):
            _,uid,_=struct.unpack('3i',self.request.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
            if uid not in self.server.allowed_uids: return
        line=self.rfile.readline(MAX_REQUEST+1)
        try:
            if len(line)>MAX_REQUEST or not line.endswith(b'\n'): raise ValueError('请求过长')
            obj=json.loads(line)
            if set(obj)!={'method','args'} or not isinstance(obj['args'],dict): raise ValueError('请求格式错误')
            result=dispatch(self.server.engine,obj['method'],obj['args'])
            reply={'ok':True,'result':result}
        except Exception as exc: reply={'ok':False,'error':str(exc)}
        self.wfile.write((json.dumps(reply,ensure_ascii=False,allow_nan=False)+'\n').encode())

class LocalServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads=True

class Handler(BaseHTTPRequestHandler):
    protocol_version='HTTP/1.1'
    def log_message(self, *_): pass  # maintenance is audited at the helper; no raw credentials

    def reply(self,status,body,kind='application/json; charset=utf-8',extra=None):
        if not isinstance(body,bytes): body=json.dumps(body,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(status); self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(body))); self.send_header('Cache-Control','no-store')
        # Rejecting a POST may leave its body unread. Do not reinterpret it as the
        # next request, or retain an idle worker forever after a status response.
        self.send_header('Connection','close'); self.close_connection=True
        self.send_header('X-Content-Type-Options','nosniff')
        for key,value in (extra or {}).items(): self.send_header(key,value)
        self.end_headers(); self.wfile.write(body)

    def call(self,method,args=None): return request_local(self.server.helper_socket,method,args)

    def do_GET(self):
        path=urlparse(self.path).path
        try:
            if path=='/api/status': return self.reply(200,self.call('status'))
            if path=='/api/report': return self.reply(200,self.call('report'))
            if path.startswith('/api/jobs/'): return self.reply(200,self.call('job',{'id':path.rsplit('/',1)[1]}))
            if path.startswith('/api/audit/'): return self.reply(200,self.call('audit',{'id':path.rsplit('/',1)[1]}))
            if path=='/api/events':
                self.send_response(200); self.send_header('Content-Type','text/event-stream')
                self.send_header('Cache-Control','no-store'); self.send_header('Connection','close'); self.end_headers()
                while True:
                    row=self.call('status')
                    self.wfile.write(('data: '+json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n\n').encode()); self.wfile.flush(); time.sleep(.05)
            if path=='/api/frame':
                # Fixed local URL reuses mediad's rotated PNG; no new V4L2 owner.
                with urlopen('http://127.0.0.1:8080/frame',timeout=4) as r:
                    data=r.read(10*1024*1024+1)
                if len(data)>10*1024*1024 or not data.startswith(b'\x89PNG'): raise ValueError('媒体快照不是有效PNG')
                return self.reply(200,data,'image/png')
            if path.startswith('/api/download/'):
                filename=path.rsplit('/',1)[1]
                if not (len(filename)==36 and filename.endswith('.zip') and all(c in '0123456789abcdef' for c in filename[:-4])):
                    raise ValueError('导出名称错误')
                p=self.server.data_dir/'exports'/filename
                if not p.is_file(): return self.reply(404,{'error':'导出不存在'})
                self.send_response(200); self.send_header('Content-Type','application/zip'); self.send_header('Content-Length',str(p.stat().st_size))
                self.send_header('Content-Disposition','attachment; filename='+filename); self.end_headers()
                with p.open('rb') as f:
                    while chunk:=f.read(65536): self.wfile.write(chunk)
                return
            if path=='/': path='/index.html'
            p=(STATIC/path.lstrip('/')).resolve()
            if not p.is_relative_to(STATIC.resolve()) or not p.is_file(): return self.reply(404,{'error':'不存在'})
            kind='text/javascript' if p.suffix=='.mjs' else mimetypes.guess_type(str(p))[0] or 'application/octet-stream'
            self.reply(200,p.read_bytes(),kind)
        except (BrokenPipeError,ConnectionResetError): return
        except Exception as exc: self.reply(503,{'error':str(exc)})

    def do_POST(self):
        try:
            # Custom header + exact Origin validation prevents browser cross-site writes.
            # This is a private-LAN maintenance surface, never a public reverse proxy.
            origin=self.headers.get('Origin'); host=self.headers.get('Host')
            if self.headers.get('X-Observer')!='1' or (origin and urlparse(origin).netloc!=host):
                return self.reply(403,{'error':'需要同源门户操作'})
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=MAX_REQUEST: return self.reply(413,{'error':'请求长度错误'})
            obj=json.loads(self.rfile.read(size)); path=urlparse(self.path).path
            if path!='/api/action': return self.reply(404,{'error':'未知接口'})
            return self.reply(202,self.call('action',obj))
        except Exception as exc: return self.reply(400,{'error':str(exc)})

def main():
    p=argparse.ArgumentParser(); p.add_argument('--helper',action='store_true'); p.add_argument('--offline',action='store_true')
    p.add_argument('--socket',default='/run/duck-observer/helper.sock'); p.add_argument('--data-dir',default='/var/lib/duck-observer')
    p.add_argument('--config',default='/etc/robot/robotd.toml'); p.add_argument('--web-uid',type=int,default=os.getuid())
    p.add_argument('--host',default='0.0.0.0'); p.add_argument('--port',type=int,default=8766); a=p.parse_args()
    if a.helper:
        path=Path(a.socket); path.parent.mkdir(parents=True,exist_ok=True)
        if path.exists():
            try:
                request_local(path,'status'); raise RuntimeError('已有辅助器，拒绝覆盖socket')
            except (ConnectionRefusedError,FileNotFoundError): path.unlink()
        engine=Engine(a.data_dir,a.config,a.offline)
        server=LocalServer(a.socket,HelperHandler); server.engine=engine; server.allowed_uids={0,a.web_uid,os.getuid()}
        os.chmod(a.socket,0o660); engine.start()
    else:
        server=ThreadingHTTPServer((a.host,a.port),Handler); server.daemon_threads=True
        server.helper_socket=a.socket; server.data_dir=Path(a.data_dir); engine=None
    def stop(*_): threading.Thread(target=server.shutdown,daemon=True).start()
    signal.signal(signal.SIGTERM,stop); signal.signal(signal.SIGINT,stop)
    try: server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        if engine:
            engine.close(); Path(a.socket).unlink(missing_ok=True)

if __name__=='__main__': main()
