#!/usr/bin/env python3
#coding=utf-8
import numpy as np
from numpy import linalg as LA

from pydrake.common import FindResourceOrThrow
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.systems.analysis import Simulator
from pydrake.all import MultibodyPlant
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.math as pm



# plant_f = MultibodyPlant(0.0)
# origarm_file = FindResourceOrThrow("drake/manipulation/models/origarm/origarmRigidKEdrake.sdf")
# origarm = Parser(plant_f).AddModelFromFile(origarm_file)

# # Define some short aliases for frames.
# W = plant_f.world_frame()
# L0 = plant_f.GetFrameByName("plate_-1", origarm)
# L5 = plant_f.GetFrameByName("plate_5", origarm)

# plant_f.WeldFrames(W, L0)
# plant_f.Finalize()

# # Allocate float context to be used by evaluators.
# context_f = plant_f.CreateDefaultContext()
# # Create AutoDiffXd plant and corresponding context.
# plant_ad = plant_f.ToAutoDiffXd()
# context_ad = plant_ad.CreateDefaultContext()

# def resolve_frame(plant, F):
#     """Gets a frame from a plant whose scalar type may be different."""
#     return plant.GetFrameByName(F.name(), F.model_instance())

# # Define target position.
# p_WT = [0.2, 0.2, 0.35]

# def link_5_distance_to_target(q):
#     """Evaluates squared distance between L7 origin and target T."""
#     # Choose plant and context based on dtype.
#     if q.dtype == float:
#         plant = plant_f
#         context = context_f
#     else:
#         # Assume AutoDiff.
#         plant = plant_ad
#         context = context_ad
#     # Do forward kinematics.
#     plant.SetPositions(context, origarm, q)
#     X_WL5 = plant.CalcRelativeTransform(
#         context, resolve_frame(plant, W), resolve_frame(plant, L5))
#     p_TL5 = X_WL5.translation() - p_WT
#     return p_TL5.dot(p_TL5)

# # WARNING: If you return a scalar for a constraint, or a vector for
# # a cost, you may get the following cryptic error:
# # "Unable to cast Python instance to C++ type"
# link_5_distance_to_target_vector = lambda q: [link_5_distance_to_target(q)]


# prog = MathematicalProgram()

# q = prog.NewContinuousVariables(plant_f.num_positions())
# # Define nominal configuration.
# q0 = np.zeros(plant_f.num_positions())

# # Add basic cost. (This will be parsed into a QuadraticCost.)
# prog.AddCost((q - q0).dot(q - q0))

# # Add constraint based on custom evaluator.
# prog.AddConstraint(
#     link_5_distance_to_target_vector,
#     lb=[0.1], ub=[0.2], vars=q)


# result = Solve(prog, initial_guess=q0)

# print(f"Success? {result.is_success()}")
# print(result.get_solution_result())
# q_sol = result.GetSolution(q)
# print(q_sol)

# print(f"Initial distance: {link_7_distance_to_target(q0):.3f}")
# print(f"Solution distance: {link_7_distance_to_target(q_sol):.3f}")



##################################################################################################
##################################################################################################
##################################################################################################

segN=2

prog = MathematicalProgram()
 
qstate = prog.NewContinuousVariables(3*segN)


# Define nominal configuration.
qstate0 = np.zeros(3*segN)
for i in range(segN):
    qstate0[i*3+2]=0.055

qstated = np.zeros(3*segN)
for i in range(segN):
    qstated[i*3+2]=0.055



#q=[a b l]
def q2Tseg(q):
    ca=pm.cos(q[0])
    cb=pm.cos(q[1])
    sa=pm.sin(q[0])
    sb=pm.sin(q[1])
    T=np.array([[ca-sb*sb*(ca-1),   cb*sb*(ca-1),   cb*sa,  q[2]*pm.sin(q[0]/2.0)*pm.cos(q[1])],
                [cb*sb*(ca-1),   ca-cb*cb*(ca-1),   sb*sa,  q[2]*pm.sin(q[0]/2.0)*pm.sin(q[1])],
                [-cb*sa,                  -sb*sa,      ca,  q[2]*pm.cos(q[0]/2.0)],
                [0,                            0,       0,                      1]])
    return T

#q=qstate
def q2T(q):
    T=1
    for i in range(segN):
        T=T*q2Tseg(q[i*3:i*3+3])
    return T

def costFuncSeg(q):
    T=q2Tseg(q)
    Td=q2Tseg(qstated)
    val=0
    for i in range(4):
        for j in range(4):
            val=val+(T[i,j]-Td[i,j])**2
    return val

def costFunc(q):
    T=q2T(q)
    Td=q2T(qstated)
    val=0
    for i in range(4):
        for j in range(4):
            val=val+(T[i,j]-Td[i,j])**2
    return val

def costjointBalance(q):
    val=np.dot((q-qstate0),(q-qstate0))/10000
    return val


prog.AddCost(costFunc,vars=qstate)
#prog.AddCost(costjointBalance,vars=qstate)


for i in range(segN):
    prog.AddBoundingBoxConstraint(0, np.pi/2.0, qstate[3*i])
    prog.AddBoundingBoxConstraint(-np.pi, np.pi, qstate[3*i+1])
    prog.AddBoundingBoxConstraint(0.035, 0.08, qstate[3*i+2])

def qdRand():
    for i in range(segN):
        qstated[i*3]=np.random.rand()*np.pi/2.0
        qstated[i*3+1]=(np.random.rand()*2-1)*np.pi
        qstated[i*3+2]=0.05+(np.random.rand()*2-1)*0.02

def qdSeq():
    for i in range(segN):
        qstated[i*3]=qstated[i*3]+0.05*np.random.rand()
        qstated[i*3+1]=qstated[i*3+1]+0.05*np.random.rand()
        qstated[i*3+2]=qstated[i*3+2]+(np.random.rand()*2-1)*0.002
        if qstated[i*3]>np.pi/2.0:
            qstated[i*3]=0
        if qstated[i*3+1]>np.pi:
            qstated[i*3+1]=-np.pi
        if qstated[i*3+2]>0.08:
            qstated[i*3+2]=0.038
np.set_printoptions(precision=4)

nbad=0
ngood=0
optiget=0
qguess=qstate0
testN=10

for i in range(testN):
   
    # qguess=qstated+np.random.rand()*0.1
    result = Solve(prog, initial_guess=qguess)
    q_sol = result.GetSolution(qstate)
    # qguess=q_sol
    
    print(qstated)
    print(q_sol)
    errv=(qstated-q_sol)
    errv=np.round(errv,decimals=2)
    # print(errv)
    err=0

    for i in range(3*segN):
        err=err+abs(errv[i])/np.pi*180
    if err>5*segN:
        print("Bad. Error:",err)
        nbad=nbad+1
    else:
        print("Good. Error:",err)
        ngood=ngood+1

    t1=q2T(qstated)
    ts=q2T(q_sol)
    eeee=t1-ts
    # print("t")
    # print(t1)
    # print("ts")
    # print(ts)
   # eeee=np.round(eeee,decimals=2)
    # print(eeee)
    ss=abs(np.sum(eeee))
    # print(ss)

    if ss<0.1:
        optiget=optiget+1
    print("")

    qdSeq()

print("")      
rate=ngood/testN
opra=optiget/testN
print("Good Rate: %",rate*100)
print("Optim get: %",opra*100)
print("")