"""Read-only connected-component audit of the supplied binary CAD mesh."""
import json
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def inspect(path):
    triangles=np.fromfile(path,dtype=np.dtype([("normal","<f4",3),("vertices","<f4",(3,3)),("attribute","<u2")]),offset=84)
    vertices=triangles["vertices"].reshape(-1,3)
    # Weld exporter round-off only: 0.0001 mm = 0.1 micrometre.
    unique,inverse=np.unique(np.round(vertices,4),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3)
    a=np.concatenate([faces[:,0],faces[:,1],faces[:,2]])
    b=np.concatenate([faces[:,1],faces[:,2],faces[:,0]])
    graph=coo_matrix((np.ones(len(a)),(a,b)),shape=(len(unique),len(unique))).tocsr()
    count,labels=connected_components(graph,directed=False)
    face_labels=labels[faces[:,0]]
    sizes=np.bincount(face_labels)
    components=[]
    for component in np.argsort(sizes)[::-1]:
        selected=np.flatnonzero(face_labels==component)
        points=triangles["vertices"][selected].reshape(-1,3)
        components.append({"id":int(component),"triangles":len(selected),
          "min_mm":points.min(axis=0).tolist(),"max_mm":points.max(axis=0).tolist(),
          "center_mm":((points.min(axis=0)+points.max(axis=0))/2).tolist(),
          "face_runs":int(np.count_nonzero(np.diff(selected)!=1)+1),
          "first_face":int(selected[0]),"last_face":int(selected[-1])})
    print(json.dumps({"components":count,"largest":components[:30]},indent=2))


if __name__=="__main__":
    inspect(Path(__file__).parent/"assets/drone_v2.stl")
