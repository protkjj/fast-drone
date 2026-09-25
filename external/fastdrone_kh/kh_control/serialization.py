"""JSON conversion and unchanged Bryson weights used in the published experiment."""
import numpy as np


def jsonable(obj):
    if isinstance(obj, dict):
        return {str(k): jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [jsonable(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return jsonable(obj.tolist())
    if isinstance(obj, (np.integer, np.floating, np.bool_)):
        return obj.item()
    return obj


def lqr_weights(p):
    tolerances = np.r_[.5, [2.]*3, [np.radians(15)]*3, [2.]*3, [.1*p['n_max']]*4]
    return np.diag(1/tolerances**2), np.eye(4)/(.15*p['n_max'])**2
