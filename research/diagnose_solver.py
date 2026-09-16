"""Bounded native IPOPT diagnostics; does not change flight/controller settings."""
import json
import time
import casadi as ca
import numpy as np
from research.model import profile, build, initial_state
from research.nmpc import build_problem, N


def main():
    p=profile("selected")
    problem,meta=build_problem(p,build(p),"nmpc",actuator_constraints=True)
    x0=initial_state(p)[:17]
    refs=np.array([[0,0,0,20]]*N+[[3,0,0,20]]).ravel()
    parameters=np.concatenate([x0,refs,meta["hover"],[0,0,0,25.2]])
    guess=np.concatenate([np.tile(x0/np.asarray(meta["state_scaling"]),N+1),
                          np.tile(np.asarray(meta["hover"])/meta["command_scaling"],N)])
    evaluate=ca.Function("nlp_values",[problem["x"],problem["p"]],[problem["f"],problem["g"]])
    initial=evaluate(guess,parameters)
    initial_constraints=np.asarray(initial[1]).ravel()
    initial_violation=np.maximum(0,np.maximum(np.asarray(meta["lbg"])-initial_constraints,
                                              initial_constraints-np.asarray(meta["ubg"])))
    print("initial cost/constraint violation",float(initial[0]),
          float(np.max(initial_violation*np.asarray(meta["constraint_scaling"]))),flush=True)
    for method,iterations in [("exact",30),("limited-memory",30)]:
        opts={"ipopt.print_level":0,"print_time":False,"ipopt.sb":"yes",
              "ipopt.max_iter":iterations,"ipopt.tol":1e-5,"ipopt.mu_strategy":"adaptive",
              "ipopt.hessian_approximation":method,"error_on_fail":False}
        solver=ca.nlpsol("diagnostic","ipopt",problem,opts)
        begin=time.perf_counter()
        result=solver(x0=guess,p=parameters,lbx=meta["lbx"],ubx=meta["ubx"],lbg=meta["lbg"],ubg=meta["ubg"])
        stats=solver.stats()
        constraints=np.asarray(result["g"]).ravel()
        violation=np.maximum(np.asarray(meta["lbg"])-constraints,constraints-np.asarray(meta["ubg"]))
        physical_defect=np.maximum(0,violation)*np.asarray(meta["constraint_scaling"])
        controls=np.asarray(result["x"])[meta["state_count"]:].ravel()*np.tile(meta["command_scaling"],N)
        print(json.dumps({"method":method,"limit":iterations,"status":stats["return_status"],
          "iterations":stats["iter_count"],"seconds":time.perf_counter()-begin,"cost":float(result["f"]),
          "physical_defect":float(np.max(np.abs(physical_defect))),"u0":controls[:4].tolist(),
          "defect_by_state":np.max(np.abs(physical_defect[:17*(N+1)].reshape(N+1,17)),axis=0).tolist(),
          "final_primal":stats.get("iterations",{}).get("inf_pr",[])[-5:],
          "final_dual":stats.get("iterations",{}).get("inf_du",[])[-5:]}),flush=True)


if __name__=="__main__":
    main()
