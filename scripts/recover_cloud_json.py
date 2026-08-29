#!/usr/bin/env python3
"""Recover a browser cloud snapshot as an XYZ binary PCD after mapping stopped."""
import json, os, struct, sys
from pathlib import Path

source=Path(sys.argv[1]);target=Path(sys.argv[2])
data=json.loads(source.read_text(encoding='utf-8'));values=data.get('points',[])
if len(values)<3 or len(values)%3:raise SystemExit('cloud JSON has no complete XYZ points')
count=len(values)//3
header=(f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\n'
        f'SIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {count}\nHEIGHT 1\n'
        f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {count}\nDATA binary\n').encode('ascii')
temporary=Path(str(target)+'.tmp');target.parent.mkdir(parents=True,exist_ok=True)
with open(temporary,'wb') as handle:
    handle.write(header)
    for offset in range(0,len(values),3):handle.write(struct.pack('<fff',*map(float,values[offset:offset+3])))
os.replace(temporary,target)
print(json.dumps({'pcd':str(target),'points':count,'source_total_points':data.get('total_points'),'source_updated_at':data.get('updated_at')}))
meta={'name':target.stem,'recovered_from_browser_cache':True,'source_total_points':data.get('total_points'),
      'source_updated_at':data.get('updated_at'),'session_id':'recovered-interrupted-20260828'}
target.with_suffix('.meta.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
if len(sys.argv)>3:
    checkpoint_file=Path(sys.argv[3]);points=json.loads(checkpoint_file.read_text(encoding='utf-8'))
    for point in points:
        if point.get('created_at','').startswith('2026-08-28 08:'):
            point['map_name']=target.stem;point['session_id']=meta['session_id'];point['recovered']=True
    temporary=Path(str(checkpoint_file)+'.tmp')
    temporary.write_text(json.dumps(points,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');os.replace(temporary,checkpoint_file)
