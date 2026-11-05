"""CoACD decomposition, with whole-source-triangle conservative re-enclosure.

CoACD supplies the spatial partition. Its approximation is NOT accepted as a
safety boundary: assign each original triangle to a part, then enclose ALL
three of its vertices in that part's outer support polytope. Convexity ensures
the complete triangle is covered. No source triangle or internal surface is
discarded by preprocessing or decimation.
"""
import hashlib
import importlib.metadata
import json
from pathlib import Path
import coacd
import numpy as np
import trimesh
from gripper_collision_geometry import outer_polytope

SETTINGS=dict(threshold=.05,max_convex_hull=8,preprocess_mode='auto',resolution=1000,
              mcts_nodes=10,mcts_iterations=20,seed=262)


def decompose(original,name,output,cache,tight=False):
    assert importlib.metadata.version('coacd')=='1.0.14'
    cache.mkdir(parents=True,exist_ok=True)
    key=hashlib.sha256(original.vertices.tobytes()+original.faces.tobytes()+json.dumps(SETTINGS,sort_keys=True).encode()).hexdigest()
    path=cache/(key+'.npz')
    if path.is_file():
        with np.load(path) as data:parts=[data[k] for k in sorted(data.files)]
    else:
        coacd.set_log_level('warn')
        partition=original.copy();partition.merge_vertices();partition.remove_unreferenced_vertices()
        if len(partition.faces)>2000:
            partition=partition.simplify_quadric_decimation(face_count=2000)
        raw=coacd.run_coacd(coacd.Mesh(partition.vertices,partition.faces),**SETTINGS)
        parts=[np.asarray(v) for v,f in raw]
        np.savez_compressed(path,**{f'part{i:04d}':v for i,v in enumerate(parts)})
    equations=[]
    for vertices in parts:
        h=trimesh.convex.convex_hull(vertices)
        normals=h.face_normals
        equations.append((normals,np.sum(h.triangles_center*normals,axis=1)))
    centers=original.triangles_center
    # Center proximity only chooses a partition; exact enclosure below adds
    # the whole triangle even when its center lies across a cut or a cavity.
    assignments=[]
    for block in np.array_split(centers,max(1,len(centers)//1000)):
        scores=np.column_stack([np.max(block@n.T-support,axis=1) for n,support in equations])
        assignments.extend(np.argmin(scores,axis=1))
    assignments=np.array(assignments)
    files=[];proof=[]
    for i in range(len(parts)):
        selected=np.flatnonzero(assignments==i)
        if not len(selected):continue
        vertices=np.unique(original.triangles[selected].reshape(-1,3),axis=0)
        if tight:
            # CoACD chooses only the partition. Use the actual supporting
            # planes, not a fixed set of directions which fills concavities.
            # Minkowski sum with a 0.1 mm cube adds >=0.1 mm in EVERY normal
            # direction and <=sqrt(3)*0.1 mm to the exact part convex hull.
            from itertools import product
            exact=trimesh.convex.convex_hull(vertices)
            offsets=np.array(list(product((-.0001,.0001),repeat=3)))
            hull=trimesh.convex.convex_hull((exact.vertices[:,None,:]+offsets).reshape(-1,3))
            normals=hull.face_normals;support=np.sum(hull.triangles_center*normals,axis=1)
        else:
            hull,normals,support=outer_polytope(vertices,.0001)
        excess=max(float(np.max(block@normals.T-support)) for block in np.array_split(vertices,max(1,len(vertices)//1000)))
        assert excess<0
        filename=f'{name}-collision-{i}.stl';hull.export(output/filename);files.append(filename)
        proof.append(dict(file=filename,source_triangles=len(selected),faces=len(hull.faces),max_vertex_excess_m=excess))
    assert sum(p['source_triangles'] for p in proof)==len(original.faces)
    return files,dict(method='CoACD_then_original_whole_triangle_'+('exact_hull_cube_offset' if tight else 'outer_enclosure'),
        coacd_version='1.0.14',settings=SETTINGS,partition_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        reserve_m=.0001,source_triangles=len(original.faces),parts=proof)
