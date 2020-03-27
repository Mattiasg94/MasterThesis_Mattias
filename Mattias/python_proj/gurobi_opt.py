from gurobipy import *
import numpy as np
import math
def optimizer(xk,yk,thk,obs,r,N,du,time_limit,Method):
    m = Model('model')
    m.setParam(GRB.param.TimeLimit,time_limit)
    m.setParam(GRB.param.NonConvex,2) 
    m.setParam(GRB.param.Method,Method)
    m.setParam(GRB.param.MIPGapAbs,100)
    m.setParam(GRB.param.FeasibilityTol,math.pow(10,-4))
    m.setParam(GRB.param.PSDTol ,math.pow(10,-3))
    m.setParam(GRB.param.OutputFlag,0)
    m.setParam(GRB.param.BarConvTol ,math.pow(10,-10))
    m.setParam(GRB.param.BarConvTol ,math.pow(10,-10))
    m.setParam(GRB.param.BarQCPConvTol ,math.pow(10,-8))
    # vars
    u_states=['v','w']
    u=m.addVars(u_states,range(N),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='u')
    states=['x','y','th']
    q=m.addVars(states,range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='q')
    qt=m.addVars(states,range(1,N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='qt')

    sin=m.addVars(range(N),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='sin')
    cos=m.addVars(range(N),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='cos')
    # A1=m.addVars(range(N),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='A1')
    # A2=m.addVars(range(N),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='A2')

    s=m.addVars(range(N+1),vtype=GRB.BINARY, name='s')
    dx=m.addVars(range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='dx')
    dy=m.addVars(range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='dy')
    rterm=m.addVars(range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='rterm')
    uterm1=m.addVars(range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='uterm1')
    uterm2=m.addVars(range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='uterm2')
    lterm=m.addVars(range(N+1),lb=-GRB.INFINITY,vtype=GRB.CONTINUOUS, name='lterm')

    # init constraints
    m.addConstr(q['x',0]==xk)
    m.addConstr(q['y',0]==yk)
    m.addConstr(q['th',0]==thk)
    # lower bound
    for state in states:
        for k in range(N):
            if state=='x' or state=='y':
                m.addConstr(q[state,k]>=0)
                m.addConstr(q[state,k]<=10)
            else:
                m.addConstr(q[state,k]>=-1)
                m.addConstr(q[state,k]<=1)
    for inp in u_states:
        for k in range(N):
            if inp=='v':
                m.addConstr(u[inp,k]>=-1)
                m.addConstr(u[inp,k]<=1)
            else:
                m.addConstr(u[inp,k]>=-1)
                m.addConstr(u[inp,k]<=1)
                
    # constraints
    for k in range(N):
        m.addGenConstrSin(q['th',k],sin[k],options="FuncPieces=-2 FuncPieceError=0.1")
        m.addGenConstrCos(q['th',k],cos[k],options="FuncPieces=-2 FuncPieceError=0.1")
    for k in range(N):
        m.addConstr(q['x',k+1]==q['x',k]+0.5*cos[k]*u['v',k])
        m.addConstr(q['y',k+1]==q['y',k]+0.5*sin[k]*u['v',k])
        m.addConstr(q['th',k+1]==q['th',k]+0.5*u['w',k])

    for state in states:
        for k in range(1,N+1):
            if state=='x':
                m.addConstr(qt[state,k]==q[state,k]-10)
            elif state=='y':
                m.addConstr(qt[state,k]==q[state,k]-5)
            else:
                m.addConstr(qt[state,k]==q[state,k])

    for k in range(N-1):
        m.addConstr(u['v',k]-u['v',k+1]>=-du[0])
        m.addConstr(u['v',k]-u['v',k+1]<=du[0])
        m.addConstr(u['w',k]-u['w',k+1]>=-du[1])
        m.addConstr(u['w',k]-u['w',k+1]<=du[1])

    # for k in range(N+1):
    #     m.addConstr(((q['x',k]-obs[0])*(q['x',k]-obs[0]))+((q['y',k]-obs[1])*(q['y',k]-obs[1]))>=r)

    # for k in range(N):
    #     m.addQConstr(dx[k]==u['v',k]*cos[k])
    #     m.addQConstr(dy[k]==u['v',k]*sin[k])
    #     m.addConstr(d[k]==((q['x',k]-obs[0])*(q['x',k]-obs[0]))+((q['y',k]-obs[1])*(q['y',k]-obs[1])))
    #     m.addQConstr(d[k]*dy[k]==dx[k]*1)

    M=10000
    for k in range(N):
        # m.addConstr((obs[0]-q['x',k])-M*s[k]<=0)
        # m.addConstr((q['x',k]-obs[0])-M*(1-s[k])<=0)
        m.addQConstr(rterm[k]==((q['x',k]-obs[0])*(q['x',k]-obs[0]))+((q['y',k]-obs[1])*(q['y',k]-obs[1])))
        m.addQConstr(dx[k]==u['v',k]*cos[k])
        m.addQConstr(dy[k]==u['v',k]*sin[k])
        m.addQConstr(uterm1[k]==
            ((dx[k]-0.001)*(q['x',k]-obs[0])+(dy[k]-0)*(q['y',k]-obs[1]))
        )
        m.addQConstr(uterm2[k]==
            ((dx[k]-0.001)*(q['x',k]-obs[0])+(dy[k]-0)*(q['y',k]-obs[1]))
        )
        m.addQConstr(lterm[k]==(dx[k]-0.001)*(dx[k]-0.001)+(dy[k]-0)*(dy[k]-0))
        m.addQConstr(r*r*lterm[k]<=rterm[k]*lterm[k]-uterm1[k]*uterm2[k])


    # obj
    qt_lst=[qt[state,k] for k in range(1,N+1) for state in states] 
    VQ=np.array(())
    for i in range(N-1):
        VQ=np.concatenate((VQ,[1,1,0.1]))
    VQ=np.concatenate((VQ,[10,10,0.1]))
    MQ=np.diag(VQ)
    m.setMObjective(MQ, None, 0.0, qt_lst, qt_lst, None, GRB.MINIMIZE)
    m.optimize()
    vars = m.getVars()
    v=[]
    w=[]
    x=[]
    y=[]
    th=[]
 
    try:
        for i in vars:
            #print(i)
            if 'v' in i.VarName:
                v.append(i.X)
            if 'w' in i.VarName:
                w.append(i.X)
            if 'q[x' in i.VarName:
                x.append(i.X)
            if 'q[y' in i.VarName:
                y.append(i.X)
            if 'q[th' in i.VarName:
                th.append(i.X)
            # if 's[' in i.VarName and not 'cos' in i.VarName:
            #     print(i)            
        # print('x',x)
        # print('y',y)
        # print('th',th)
        return x,y,th,v[0],w[0]
    except:
        print("NOT FIESEEBLE")
        return 11,11,0,0,0
#optimizer(4,5,-0.5)
