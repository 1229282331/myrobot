"""Registered session files only. Bounded text polling never touches control IO."""
import base64
import hashlib
import json
import os
from pathlib import Path
from uuid import uuid4

CHUNK_BYTES=64*1024
SOURCES=('control','pc','camera','servo')


class LogSources:
    def __init__(self,session=None):
        self.session=session or uuid4().hex
        self.files={}

    def register(self,name,path,*,root=None):
        if name not in SOURCES:raise ValueError('Unknown log source')
        self.files[name]=(path,Path(root).resolve() if root else None)

    def path(self,name):
        if name not in self.files:raise ValueError('Unregistered log source')
        source,root=self.files[name]
        source=source() if callable(source) else source
        if source is None:raise FileNotFoundError('Log source is not running')
        path=Path(source).resolve()
        if root and not path.is_relative_to(root):raise ValueError('Log source escaped its session root')
        if not path.is_file():raise FileNotFoundError('Log source has not produced output yet')
        return path

    def listing(self):
        result=[]
        for name in self.files:
            try:self.path(name);available=True
            except (OSError,ValueError):available=False
            result.append(dict(id=name,label='PC' if name=='pc' else name.title(),available=available))
        return dict(session=self.session,sources=result)

    @staticmethod
    def _anchor(stream,offset):
        stream.seek(max(0,offset-64))
        return hashlib.sha256(stream.read(min(64,offset))).hexdigest()

    def read(self,name,cursor=''):
        previous=None
        if cursor:
            if len(cursor)>1024:raise ValueError('Invalid log cursor')
            try:previous=json.loads(base64.urlsafe_b64decode(cursor.encode()))
            except Exception as exc:raise ValueError('Invalid log cursor') from exc
            if (not isinstance(previous,list) or len(previous)!=3 or type(previous[1]) is not int
                or previous[1]<0):raise ValueError('Invalid log cursor')
        with self.path(name).open('rb') as stream:
            stat=os.fstat(stream.fileno())
            identity=f'{self.session}:{name}:{stat.st_dev}:{stat.st_ino}'
            reset=previous is None or previous[0]!=identity or previous[1]>stat.st_size
            if not reset:reset=self._anchor(stream,previous[1])!=previous[2]
            # A new/rotated file starts with a bounded recent window.
            offset=max(0,stat.st_size-CHUNK_BYTES) if reset else previous[1]
            stream.seek(offset)
            data=stream.read(CHUNK_BYTES)
            if reset and offset:
                # UTF-8 continuation bytes at the beginning are not a character.
                while data and data[0]&0xC0==0x80:offset+=1;data=data[1:]
            end=offset+len(data)
            token=base64.urlsafe_b64encode(json.dumps([identity,end,self._anchor(stream,end)]).encode()).decode()
        return data,{'X-Log-Cursor':token,'X-Log-Reset':'1' if reset else '0','Cache-Control':'no-store'}

    def download(self,name):
        stream=self.path(name).open('rb')
        return stream,os.fstat(stream.fileno()).st_size
