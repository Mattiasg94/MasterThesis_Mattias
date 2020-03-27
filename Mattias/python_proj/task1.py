from gurobipy import *
from csvTolist import csvToLists
m = Model('model')


J = [1, 2, 3]
mch = [1, 2, 3, 4]
pt = [[10, 8, 4], [8, 3, 5, 6], [4, 7, 3]]
mch_seq = [[1, 2, 3], [2, 1, 4, 3], [1, 2, 4]]

J, pt, mch_seq, mch = csvToLists(r"cvs_files\ft06.csv")

M = 0
for lst in pt:
    M += sum(lst)


def getJobsMachineIsIn(machine):
    job_lst = []
    for i in range(len(J)):
        if machine in mch_seq[i]:
            job_lst.append(i+1)
    return job_lst


# create list to be used in mutal exclusion constraint
def getCombinationsOfJobs(jobs):
    lst_raw = [(x, y) for x in jobs for y in jobs]
    lst = []
    for el in lst_raw:
        if not el[0] == el[1]:
            lst.append(el)
    lst = {tuple(sorted(item)) for item in lst}
    return lst


def getNumOfX():
    num_x = 0
    for i in mch:
        jobs = getJobsMachineIsIn(i)
        lst_jobs = getCombinationsOfJobs(jobs)
        num_x += len(lst_jobs)
    return num_x


num_x = getNumOfX()
list_x = [i for i in range(num_x)]

t = m.addVars(J, mch, vtype=GRB.INTEGER, name='t')
X = m.addVars(list_x, vtype=GRB.BINARY, name='x')
z = m.addVar(vtype=GRB.INTEGER, name='z')

for job in range(len(J)):  # for every job
    for i in range(len(mch_seq[job])-1):
        m.addConstr(pt[job][i]+t[J[job], mch_seq[job][i]]
                    <= t[J[job], mch_seq[job][i+1]])


# for all machine
x_index = 0
for i in mch:
    jobs = getJobsMachineIsIn(i)
    lst_jobs = getCombinationsOfJobs(jobs)
    print(lst_jobs)
    for comb in lst_jobs:
        # I=index of machine to be used to get "pt"
        I0 = mch_seq[comb[0]-1].index(i)
        I1 = mch_seq[comb[1]-1].index(i)

        m.addConstr(
            t[comb[0], i] >= t[comb[1], i] + pt[comb[1]-1][I1]-M * X[x_index]
        )
        m.addConstr(
            t[comb[1], i] >= t[comb[0], i] + pt[comb[0]-1][I0]-M*(1-X[x_index])
        )
        x_index += 1


for job in J:
    m.addConstr(t[job, mch_seq[job-1][-1]] + pt[job-1][-1] <= z)

m.setObjective(z, GRB.MINIMIZE)

m.optimize()
vars = m.getVars()
for i in vars:
    print(i)
