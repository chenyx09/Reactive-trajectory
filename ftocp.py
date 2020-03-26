from casadi import *
from numpy import *
import pdb
import itertools
import numpy as np

##### MY CODE ######
class FTOCP(object):
	""" Finite Time Optimal Control Problem (FTOCP)
	Methods:
		- solve: solves the FTOCP given the initial condition x0 and terminal contraints
		- buildNonlinearProgram: builds the nonlinear program solved by the above solve methos
		- model: given x_t and u_t computes x_{t+1} = Ax_t + Bu_t
	"""
	def __init__(self, N, dt):
		# Define variables
		self.N    = N
		self.dt   = dt
		self.n    = 4
		self.d    = 2
		self.vRef = 30
		self.aEll = 4
		self.bEll = 1
		

	def solve(self, x0):

		# Set box constraints on states, input and obstacle slack
		self.lbx = x0.tolist() + [-1000]*(self.n*(self.N)) + [-np.pi/2.0,-1.0]*self.N + [1]*(self.N*self.tr_num)
		self.ubx = x0.tolist() +  [1000]*(self.n*(self.N)) + [ np.pi/2.0, 1.0]*self.N + [100000]*(self.N*self.tr_num)
		

		# Solve nonlinear programm
		# self.xGuessTot = np.concatenate((self.xGuess, np.zeros(self.n+self.N-1)), axis=0)
		# sol = self.solver(lbx=self.lbx, ubx=self.ubx, lbg=self.lbg_dyanmics, ubg=self.ubg_dyanmics, x0 = self.xGuessTot.tolist())
		sol = self.solver(lbx=self.lbx, ubx=self.ubx, lbg=self.lbg_dyanmics, ubg=self.ubg_dyanmics)

		# Store optimal solution
		x = np.array(sol["x"])
		self.xSol  = x[0:(self.N+1)*self.n].reshape((self.N+1,self.n)).T
		self.uSol  = x[(self.N+1)*self.n:((self.N+1)*self.n + self.d*self.N)].reshape((self.N,self.d)).T
		self.inVar = x[((self.N+1)*self.n + self.d*self.N):((self.N+1)*self.n + self.d*self.N + ((self.N-1)*self.tr_num) )]

		# Check solution flag
		if (self.solver.stats()['success']):
			self.feasible = 1
		else:
			self.feasible = 0

	def buildNonlinearProgram(self, xpred, ypred):
		# Define variables
		n = self.n
		d = self.d
		N = self.N
		self.tr_num = xpred.shape[0];

		X     = SX.sym('X', n*(N+1));
		U     = SX.sym('X', d*N);
		inVar = SX.sym('X', N*self.tr_num);

		# Define dynamic constraints
		constraint = []
		for i in range(0, N):
			constraint = vertcat(constraint, X[n*(i+1)+0] - (X[n*i+0] + self.dt*(X[n*i+2]*np.cos(X[n*i+3]))) );
			constraint = vertcat(constraint, X[n*(i+1)+1] - (X[n*i+1] + self.dt*(X[n*i+2]*np.sin(X[n*i+3]))) );
			constraint = vertcat(constraint, X[n*(i+1)+2] - (X[n*i+2] + self.dt*(U[d*i+1])) );
			constraint = vertcat(constraint, X[n*(i+1)+3] - (X[n*i+3] + self.dt*(U[d*i+0])) );


		# Obstacle constraints
		for j in range(0, self.tr_num ):
			for i in range(0, N):
				constraint = vertcat(constraint, ( ( X[n*i+0] -  xpred[j,i] )**2/(self.aEll**2) + ( X[n*i+1] - ypred[j,i] )**2/(self.bEll**2)  - inVar[ j*N + i] ) );

		# Defining Cost
		cost = 0
		for i in range(0, N):
			cost = cost + 0.0*X[n*i+0]**2 + 0.0*X[n*i+1]**2 + (X[n*i+2] - self.vRef)**2 + 0.0*X[n*i+3]**2 + U[d*i+0]**2 + U[d*i+1]**2;
		# Terminal cost
		cost = cost + 0.0*X[n*N+0]**2 + 0.0*X[n*N+1]**2 + (X[n*N+2] - self.vRef)**2 + 0.0*X[n*N+3]**2

		# Set IPOPT options
		# opts = {"verbose":False,"ipopt.print_level":0,"print_time":0}#, "ipopt.acceptable_constr_viol_tol":0.001}#,"ipopt.acceptable_tol":1e-4}#, "expand":True}
		# opts = {"verbose":False,"ipopt.print_level":0,"print_time":0,"ipopt.mu_strategy":"adaptive"}#, "ipopt.acceptable_constr_viol_tol":0.001}#,"ipopt.acceptable_tol":1e-4}#, "expand":True}
		opts = {"verbose":False,"ipopt.print_level":0,"print_time":0,"ipopt.mu_strategy":"adaptive","ipopt.mu_init":1e-5,"ipopt.mu_min":1e-15,"ipopt.barrier_tol_factor":1}#, "ipopt.acceptable_constr_viol_tol":0.001}#,"ipopt.acceptable_tol":1e-4}#, "expand":True}
		nlp = {'x':vertcat(X,U, inVar), 'f':cost, 'g':constraint}
		self.solver = nlpsol('solver', 'ipopt', nlp, opts)

		# Set lower bound of inequality constraint to zero to force: 1) n*N state dynamics and 2) inequality constraints (set to zero as we have slack)
		self.lbg_dyanmics = [0]*(n*N) + [0]*( N*self.tr_num )
		self.ubg_dyanmics = [0]*(n*N) + [0]*( N*self.tr_num )

	def f(self, x, u):
		# Given a state x and input u it return the successor state
		xNext = np.array([x[0] + self.dt*x[2]*np.cos(u[0]),
						  x[1] + self.dt*x[2]*np.sin(u[0]),
						  x[2] + u[1]])
		return xNext.tolist()