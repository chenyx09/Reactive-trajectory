import torch
import pdb
from affordance import *
import argparse
import torchvision.transforms as transforms
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
from scipy.io import loadmat
from torch.utils import data
from scipy import interpolate
import random
import math
from numpy import linalg as LA
import pdb
from ftocp import FTOCP
from numpy.linalg import norm
import multiprocessing

data = loadmat('data.mat')
traj_base = data['traj_base1']
x = loadmat('offset.mat')
offset = x['offset'][0]
Ts = 0.5

# N_lane = 6
lane_width = 3.6
UB = 30
LB = -10
v0 = 20
m = 7
aff_idx = [1,2,3,4,7,8,9,10,11,12,13,14,15,16,17,18,23,24,25,26,27]
tt = np.arange(0,m)*Ts
lm = [0,3.6,7.2,10.8,14.4,18,21.6]
N_base = 17
plotPredictionFlag = True

scale = np.zeros([m,2])
delta = 0.5
for i in range(0,m):
    scale[i][0]=(m-1)/(i+1e-3)/4
    scale[i][1]=(m-1)/(i+1e-3)


class FCNet(nn.Module):
    def __init__(self, hidden_1=200, hidden_2=200 ,op_dim=17, input_dim=21):
        super().__init__()
        self.fc1 = nn.Linear(input_dim, hidden_1)
        self.fc2 = nn.Linear(hidden_2, op_dim)
        self.fc3 = nn.Sigmoid()
        self.bn1 = nn.BatchNorm1d(hidden_1)

    def forward(self, x):
        output = F.relu(self.bn1(self.fc1(x)))
        output1 = self.fc2(output)
        output2 = self.fc3(output1)
        # output = F.log_softmax(output, dim=1)
        return output2, output

class FCNet_new(nn.Module):
    def __init__(self, hidden_1=100, hidden_2=50, op_dim=17, input_dim=21):
        super().__init__()
        self.fc1 = nn.Linear(input_dim, hidden_1)
        self.fc2 = nn.Linear(hidden_1, hidden_2)
        self.fc3 = nn.Linear(hidden_2, op_dim)
        self.fc4 = nn.Sigmoid()
        self.bn1 = nn.BatchNorm1d(hidden_1)
        self.bn2 = nn.BatchNorm1d(hidden_2)
    def forward(self, x):
        output = F.relu(self.bn1(self.fc1(x)))
        output = F.relu(self.bn2(self.fc2(output)))
        output1 = self.fc3(output)
        output2 = self.fc4(output1)
        # output = F.log_softmax(output, dim=1)
        return output2, output
def check_collision_simple(traj1,traj2,L,W,dt):
    N = len(traj1[0])

    for j in range(0,N):
        dis = max(abs(traj1[0][j]-traj2[0][j])-L,abs(traj1[1][j]-traj2[1][j])-W)
        if dis<0.2:
            return j*dt
    return 10
def check_collision(affordance,traj,Ts1,T):
    # 0 lane_ID(i)
    # 1 v_Vel(i)
    # 2 dis2cen
    # 3 fwd_dis
    # 4 fwd_vel
    # 5 rear_dis
    # 6 rear_vel
    # 7 left_front_Y
    # 8 left_front_X
    # 9 left_front_vel
    # 10 left_rear_Y
    # 11 left_rear_X
    # 12 left_rear_vel
    # 13 right_front_Y
    # 14 right_front_X
    # 15 right_front_vel
    # 16 right_rear_Y
    # 17 right_rear_X
    # 18 right_rear_vel
    # 19 left_fwd_Y
    # 20 left_fwd_vel
    # 21 right_fwd_Y
    # 22 right_fwd_vel
    # 23 left_front_L
    # 24 left_rear_L
    # 25 right_front_L
    # 26 right_rear_L
    # 27 vehicle length

    m = int(len(traj)/2)
    L = affordance[27]
    W = 2.4
    buffer_x = 0.1
    buffer_y = 0.1
    dis2cen = affordance[2]

    v0 = affordance[1]

    fwd_vel         = v0 + affordance[4]
    left_front_vel  = v0 + affordance[9]
    left_rear_vel   = v0 + affordance[12]
    right_front_vel = v0 + affordance[15]
    right_rear_vel  = v0 + affordance[18]

    left_front_X = affordance[8]
    left_rear_X = affordance[11]
    right_front_X = affordance[14]
    right_rear_X = affordance[17]

    left_front_L = affordance[23]
    left_rear_L = affordance[24]
    right_front_L = affordance[25]
    right_rear_L = affordance[26]

    fwd_traj         = affordance[3]+fwd_vel*tt;
    left_front_traj  = affordance[7]+left_front_vel*tt;
    left_rear_traj   = affordance[5]+left_rear_vel*tt;
    right_front_traj = affordance[13]+right_front_vel*tt;
    right_rear_traj  = affordance[16]+right_rear_vel*tt;



    y_traj = v0*tt + traj[0:m];
    x_traj = traj[m:]+dis2cen;

    TTC = 10;
    for i in range(0,m):
        if i*Ts1>T:
            break

        if x_traj[i]-buffer_x<dis2cen-left_front_X and y_traj[i]>left_front_traj[i]-buffer_y and y_traj[i]-L< left_front_traj[i]+left_front_L+buffer_y: ## left lane
            TTC = i*Ts1
            break

        if x_traj[i]-buffer_x<dis2cen-left_rear_X and y_traj[i]<left_rear_traj[i]+L+buffer_y and y_traj[i]>left_rear_traj[i]-left_rear_L-buffer_y:
            TTC = i*Ts1
            break

        if abs(x_traj[i])<1.8-buffer_x+W/4 and y_traj[i]>fwd_traj[i]-buffer_y:
            TTC = i*Ts1
            break

        if x_traj[i]+buffer_x>dis2cen+right_front_X and y_traj[i]>right_front_traj[i]-buffer_y and y_traj[i]-L< right_front_traj[i]+right_front_L+buffer_y:

            TTC = i*Ts1
            break
        if x_traj[i]+buffer_x>dis2cen+right_rear_X and y_traj[i]<right_rear_traj[i]+L+buffer_y and y_traj[i]>right_rear_traj[i]-right_rear_L-buffer_y:
            TTC = i*Ts1
            break
    return TTC






class vehicle():
    def __init__(self, state=[0,0,v0,0], controlled=False,traj_idx=0,v_length=4,v_width=2.4,ts=0.05):
        self.state = np.array(state)
        self.controlled = controlled
        self.ts = ts
        self.v_length = v_length
        self.v_width = v_width
        self.update_traj(traj_idx)
        self.x_pred = []
        self.y_pred = []
        self.psi_pred = []
        self.obs_rec_x = []
        self.obs_rec_y = []
    def controlled_step(self,u): # controlled vehicle
        if self.controlled !=True:
            pdb.set_trace()
        assert self.controlled==True

        dxdt = np.array([self.state[2]*np.cos(self.state[3]),self.state[2]*np.sin(self.state[3]),u[0],u[1]])
        self.state = self.state + dxdt*self.ts
    def uncontrolled_step(self):   #uncontrolled vehicle
        assert self.controlled==False
        self.t = self.t+1
        self.state[0] = self.Y_traj[self.t]
        self.state[1] = self.X_traj[self.t]
        self.state[2] = self.v_traj[self.t]
        # print("Update y traj")
        # print(self.Y_traj)

        # pdb.set_trace()

    def update_traj(self,traj_idx):
        self.t = 0
        self.traj_idx = traj_idx
        err = np.random.multivariate_normal([0,0], [[1/delta, 0 ],[0,1/delta/4]], m)
        for i in range(0,m):
            err_norm = norm(err[i]*scale[i])
            if err_norm>delta:
                err[i]=err[i]/err_norm*delta
        # pdb.set_trace()
        fy = interpolate.interp1d(tt, traj_base[traj_idx][0:m]+err.transpose()[0]+self.state[2]*tt+self.state[0])
        fx = interpolate.interp1d(tt, traj_base[traj_idx][m:2*m]+err.transpose()[1]+self.state[1])
        fv = interpolate.interp1d(tt, traj_base[traj_idx][2*m:]+self.state[2])
        t_ts = np.arange(0,3+self.ts,self.ts)
        self.Y_traj = fy(t_ts)
        self.X_traj = fx(t_ts)
        self.v_traj = fv(t_ts)

class Highway_env():
    def __init__(self,AV_state=[0,1.8,20,0],N_HV=10,N_lane=6,ts=0.05,pred_model=[]):
        self.ts = ts
        self.veh_set = [vehicle(state=AV_state, controlled=True,ts=self.ts)]
        self.N_lane = N_lane
        self.pred_model = pred_model
        self.mpc_ts = 0.25

        for i in range(0,N_HV):
            lane_number = math.floor(random.random()*N_lane)
            success = False
            while not success:
                X = (lane_number+0.5)*lane_width+np.random.normal(0,0.5)
                Y = random.random()*(UB-LB)+LB
                collision = False
                for veh in self.veh_set:
                    if abs(X-veh.state[1])<=3 and abs(Y-veh.state[0])<=8:
                        collision=True
                        break
                if not collision:
                    success = True

            self.veh_set.append(vehicle([Y,X,v0,0],controlled=False,traj_idx=0,ts=self.ts))

        # DO TO !: Set N and dt correctly!!!!!
        self.MPC_N  = int(3/self.mpc_ts)+1

        self.ftocp = FTOCP(self.MPC_N, self.mpc_ts,self.N_lane)

    def step(self):
        u=[None]*len(self.veh_set)
        for i in range(0,len(self.veh_set)):
            if self.veh_set[i].controlled==True:
                # controller implementation u[i] = controller(x)
                x = self.veh_set[i].state
                # idx = [1,2,3,4,7,8,9,10,11,12,13,14,15,16,17,18,23,24,25,26,27]
                veh_affordance=calc_affordance(self.veh_set,self.N_lane)
                pos_pred_x_tot = np.empty((0, self.MPC_N)); # TO DO replace 7
                pos_pred_y_tot = np.empty((0, self.MPC_N));
                for j in range(0,len(self.veh_set)):
                    if i!=j and abs(self.veh_set[i].state[0]-self.veh_set[j].state[0])<40 and abs(self.veh_set[i].state[1]-self.veh_set[j].state[1])<5:
                        # and not (self.veh_set[i].state[0]>self.veh_set[j].state[0]+2 and abs(self.veh_set[i].state[1]-self.veh_set[j].state[1])<3):
                        pred = self.pred_model(torch.tensor([veh_affordance[j,aff_idx]],dtype=torch.float32))[0][0].tolist()
                        possible_traj_idx = [i for i in range(len(pred)) if pred[i] > offset[i]]
                        traj = traj_base[possible_traj_idx]#+np.concatenate((np.arange(0,m)*self.veh_set[j].state[2],np.zeros(2*m)))
                        # store predicte trajectory

                        pos_pred_y0 = traj[:,0:7]  + self.veh_set[j].state[0] + self.veh_set[j].state[2]*tt
                        pos_pred_x0 = traj[:,7:14] + self.veh_set[j].state[1]

                        fy = interpolate.interp1d(tt, pos_pred_y0)
                        fx = interpolate.interp1d(tt, pos_pred_x0)
                        t_ts = np.arange(0,3+self.mpc_ts,self.mpc_ts)
                        # pdb.set_trace()
                        # fy = interpolate.interp1d(tt, traj[0:m]+self.veh_set[j].state[2]*tt+self.veh_set[j].state[0])
                        # fx = interpolate.interp1d(tt, traj[m:2*m]+self.veh_set[j].state[1])

                        pos_pred_y = fy(t_ts)
                        pos_pred_x = fx(t_ts)
                        if j==3:
                            pos_pred_y=pos_pred_y[0:4]
                            pos_pred_x=pos_pred_x[0:4]

                        pos_pred_x_tot = np.concatenate((pos_pred_x_tot, pos_pred_x), axis=0)
                        pos_pred_y_tot = np.concatenate((pos_pred_y_tot, pos_pred_y), axis=0)
                self.veh_set[i].obs_rec_x.append(pos_pred_x_tot)
                self.veh_set[i].obs_rec_y.append(pos_pred_y_tot)
                # if self.veh_set[0].state[0]==0:
                #     print(traj)
                #     print(pred)

                #
                # print("pos_pred_x shape: ", pos_pred_x_tot.shape)
                # print("pos_pred_y shape: ", pos_pred_y_tot.shape)

                self.ftocp.buildNonlinearProgram(pos_pred_x_tot, pos_pred_y_tot)

                # print("State x: ", x)

                self.ftocp.solve(x)


                # print("Optimal State Trajectory")
                # print(self.ftocp.xSol[:,0:2].T)

                # print("Optimal Input Trajectory")
                # print(self.ftocp.uSol.T)

                # print("Predicted inVar = (x-x_obs)^2/xEll^2 + (y-y_obs)^2/yEll^2")
                if len(self.ftocp.inVar)>1:
                    idx = np.argmin(self.ftocp.inVar)
                    tr_idx   = int(np.floor(idx/self.MPC_N))
                    time_idx = idx - tr_idx*self.MPC_N
                # else:
                    # print("No other vehicles")
                    # pdb.set_trace()

                # print("idx (tr,time): ", tr_idx, time_idx, " value: ", self.ftocp.inVar[idx], "Pred (x,y): ",pos_pred_x_tot[tr_idx][time_idx], pos_pred_y_tot[tr_idx][time_idx])
                # if (time_idx+1<self.MPC_N) and (idx+1 < self.ftocp.N*self.ftocp.tr_num):
                #     print("idx (tr,time): ", tr_idx, time_idx+1, " value: ", self.ftocp.inVar[idx+1], "Pred (x,y): ",pos_pred_x_tot[tr_idx][time_idx+1],pos_pred_y_tot[tr_idx][time_idx+1])

                # print("y predicted")
                # print(pos_pred_y_tot[tr_idx][:])
                # u[i]=[0.0, 0.0]
                # self.ftocp.feasible = 1

                plotFlag = False
                if (plotFlag == True) and (self.ftocp.feasible == 1):

                    fig = plt.figure()
                    ax = fig.add_subplot(111)

                    veh_patch = [];
                    veh = self.veh_set[1]

                    for ii in range(0, pos_pred_x_tot.shape[0]):
                        for jj in range(0, pos_pred_x_tot.shape[1]):
                            if jj == 0:
                                veh_patch = plt.Rectangle((pos_pred_x_tot[ii][jj]-veh.v_width/2, pos_pred_y_tot[ii][jj]-veh.v_length/2), veh.v_width, veh.v_length, fc='r', zorder=0)
                            else:
                                veh_patch = plt.Rectangle((pos_pred_x_tot[ii][jj]-veh.v_width/2, pos_pred_y_tot[ii][jj]-veh.v_length/2), veh.v_width, veh.v_length, fc='b', zorder=0)
                            ax.add_patch(veh_patch)

                    veh_patch = plt.Rectangle((self.veh_set[0].state[1]-veh.v_width/2, self.veh_set[0].state[0]-veh.v_length/2), veh.v_width, veh.v_length, fc='g', zorder=0)
                    ax.add_patch(veh_patch)
                    for ii in range(1, self.ftocp.xSol.shape[1]):
                        veh_patch = plt.Rectangle((self.ftocp.xSol[1][ii]-veh.v_width/2, self.ftocp.xSol[0][ii]-veh.v_length/2), veh.v_width, veh.v_length, fc='y', zorder=0)
                        ax.add_patch(veh_patch)

                    for jj in range(0, len(lm)):
                        plt.plot([lm[jj], lm[jj]], [-30, 1000], 'go--', linewidth=2)

                    ax.axis('equal')
                    ax.set_xlim(0, self.N_lane*lane_width)
                    ax.set_ylim(self.veh_set[0].state[0]-10, self.veh_set[0].state[0]+100)

                    print("Optimal State Trajectory")
                    print(self.ftocp.xSol.T)

                    plt.show()

                if self.ftocp.feasible == 1:
                    self.veh_set[0].x_pred.append(self.ftocp.xSol[1,:])
                    self.veh_set[0].y_pred.append(self.ftocp.xSol[0,:])
                    self.veh_set[0].psi_pred.append(self.ftocp.xSol[3,:])
                    # print("MPC problem solved to optimality")
                    self.ftocp.xPredOld= self.ftocp.xSol
                    u[i]=[self.ftocp.uSol[0][0],self.ftocp.uSol[1][0]]
                else:
                    # print('infeasible')
                    self.veh_set[0].x_pred.append(None)
                    self.veh_set[0].y_pred.append(None)
                    self.veh_set[0].psi_pred.append(None)
                    # print("Error Not Feasible")
                    # print("Size pos_pred_x_tot: ", pos_pred_x_tot.shape)
                    #
                    # fig = plt.figure()
                    # ax = fig.add_subplot(111)
                    #
                    # veh_patch = [];
                    # veh = self.veh_set[1]
                    #
                    # for ii in range(0, pos_pred_x_tot.shape[0]):
                    #     for j in range(0, pos_pred_x_tot.shape[1]):
                    #         if j == 0:
                    #             veh_patch = plt.Rectangle((pos_pred_x_tot[ii][j]-veh.v_width/2, pos_pred_y_tot[ii][j]-veh.v_length/2), veh.v_width, veh.v_length, fc='r', zorder=0)
                    #         else:
                    #             veh_patch = plt.Rectangle((pos_pred_x_tot[ii][j]-veh.v_width/2, pos_pred_y_tot[ii][j]-veh.v_length/2), veh.v_width, veh.v_length, fc='b', zorder=0)
                    #         ax.add_patch(veh_patch)
                    #
                    #
                    # veh_patch = plt.Rectangle((self.veh_set[0].state[1]-veh.v_width/2, self.veh_set[0].state[0]-veh.v_length/2), veh.v_width, veh.v_length, fc='g', zorder=0)
                    # ax.add_patch(veh_patch)
                    #
                    # for j in range(0, len(lm)):
                    #     plt.plot([lm[j], lm[j]], [-30, 1000], 'go--', linewidth=2)
                    #
                    # ax.axis('equal')
                    # ax.set_xlim(0, self.N_lane*lane_width)
                    # ax.set_ylim(self.veh_set[0].state[0]-10, self.veh_set[0].state[0]+50)
                    #
                    # if self.ftocp.xPredOld != []:
                    #     for ii in range(0, self.ftocp.xSol.shape[1]):
                    #         veh_patch = plt.Rectangle((self.ftocp.xPredOld[1][ii]-veh.v_width/2, self.ftocp.xPredOld[0][ii]-veh.v_length/2), veh.v_width, veh.v_length, fc='y', zorder=0)
                    #         ax.add_patch(veh_patch)
                    #
                    # print("Old OPT")
                    # print(self.ftocp.xSol[:,:].T)
                    # plt.show()

                    # pdb.set_trace()
                    u[i] = [0.0,0.0]

                # pdb.set_trace()


                self.veh_set[i].controlled_step(u[i])
            else:
                self.veh_set[i].uncontrolled_step()
                u[i]=[]
        return u



def animate_scenario(env,state_rec,lm,traj_base,output):
    if output:
        matplotlib.use("Agg")
    fig = plt.figure(figsize=((4+lane_width*env.N_lane)/5,10))
    ax = fig.add_subplot(111)
    plt.grid()

    nframe = len(state_rec[0])
    ego_veh = env.veh_set[0]
    veh_patch = [plt.Rectangle((ego_veh.state[1]-ego_veh.v_width/2, ego_veh.state[0]-ego_veh.v_length/2), ego_veh.v_width, ego_veh.v_length, fc='r', zorder=0)]
    ego_y = ego_veh.state[0]
    for veh in env.veh_set[1:]:
        veh_patch.append(plt.Rectangle((veh.state[1]-veh.v_width/2, veh.state[0]-veh.v_length/2), veh.v_width, veh.v_length, fc='b', zorder=0))
    for patch in veh_patch:
        ax.add_patch(patch)
    # for j in range(0, len(lm)):
    #     plt.plot([lm[j], lm[j]], [-30, 1000], 'go--', linewidth=2)

    pred_tr_patch = []
    if plotPredictionFlag == True:
        for ii in range(1, env.ftocp.xSol.shape[1]):
            pred_tr_patch.append(plt.Rectangle((env.veh_set[0].x_pred[0][ii]-veh.v_width/2, env.veh_set[0].y_pred[0][ii]-veh.v_length/2), veh.v_width, veh.v_length, fc='r',alpha=0.3, zorder=0))
        for patch in pred_tr_patch:
            ax.add_patch(patch)


    def animate(t,veh_patch,state_rec,env,pred_tr_patch):
        N_veh = len(state_rec)
        ego_y = state_rec[0][t][0]
        ax.clear()
        ax.set_xlim(-2, lane_width*env.N_lane+2)
        ax.set_ylim(ego_y-15, ego_y+35)
        ts = ax.transData
        for i in range(0,N_veh):
            coords = ts.transform([state_rec[i][t][1],state_rec[i][t][0]])
            tr = matplotlib.transforms.Affine2D().rotate_around(coords[0], coords[1], -state_rec[i][t][3])
            te = ts + tr
            veh_patch[i].set_xy([state_rec[i][t][1]-env.veh_set[i].v_width/2,state_rec[i][t][0]-env.veh_set[i].v_length/2])
            veh_patch[i].set_transform(te)
            ax.add_patch(veh_patch[i])

        for j in range(0, env.N_lane+1):
            plt.plot([lm[j], lm[j]], [-10, 1000], 'go--', linewidth=2)

        if plotPredictionFlag == True:
            if env.veh_set[0].x_pred[t] is not None:
                for ii in range(1, env.ftocp.xSol.shape[1]):
                    # coords = ts.transform([state_rec[i][t][1],state_rec[i][t][0]])
                    # tr = matplotlib.transforms.Affine2D().rotate_around(coords[0], coords[1], -state_rec[i][t][3])
                    # te= ts + tr
                    pred_tr_patch[ii-1].set_xy([env.veh_set[0].x_pred[t][ii]-veh.v_width/2, env.veh_set[0].y_pred[t][ii]-veh.v_length/2])
                    coords = ts.transform([env.veh_set[0].x_pred[t][ii],env.veh_set[0].y_pred[t][ii]])
                    tr = matplotlib.transforms.Affine2D().rotate_around(coords[0], coords[1], -env.veh_set[0].psi_pred[t][ii])
                    te = ts + tr
                    pred_tr_patch[ii-1].set_transform(te)
                    ax.add_patch(pred_tr_patch[ii-1])
            # for ii in range(0,len(env.veh_set[0].obs_rec_x[t])):
            #     for jj in range(0,len(env.veh_set[0].obs_rec_x[t][ii])):
            #         obs_patch = plt.Rectangle((env.veh_set[0].obs_rec_x[t][ii][jj]-veh.v_width/2, env.veh_set[0].obs_rec_y[t][ii][jj]-veh.v_length/2), veh.v_width, veh.v_length, fc='m', zorder=0)
            #         ax.add_patch(obs_patch)

        # ax.axis('equal')
        # ax.set_xlim(0, env.N_lane*lane_width)
        # ax.set_ylim(ego_y-10, ego_y+50)


        # return near_veh
        # print(len(ax.patches))
        # print(len(plotted_veh_ID))
        return veh_patch
    anim = animation.FuncAnimation(fig, animate, fargs=(veh_patch,state_rec,env,pred_tr_patch,),
                                   frames=nframe,
                                   interval=50,
                                   blit=False, repeat=False)


    if output:
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
        anim_name = output
        anim.save(anim_name,writer=writer)
    else:
        plt.show()

def Highway_sim(env,T):
    N_veh = len(env.veh_set)
    collision = 0  # 0 for no collision, 1 for normal collision, and 2 for rear hit
    ts = env.ts
    t=0
    Ts_update = 0.3
    N_update = int(round(Ts_update/ts))
    N = int(round(T/ts))
    state_rec = np.zeros([N_veh,N,4])
    input_rec = np.zeros([N,2])
    for i in range(0,len(env.veh_set)):
        state_rec[i][t]=env.veh_set[i].state
    L=env.veh_set[0].v_length
    W=env.veh_set[0].v_width
    while t<N:
        if collision==0:
            for i in range(1,N_veh):
                dis = max(abs(env.veh_set[0].state[0]-env.veh_set[i].state[0])-L,abs(env.veh_set[0].state[1]-env.veh_set[i].state[1])-W)
                if dis<0:
                    # pdb.set_trace()

                    if abs(env.veh_set[0].state[1]-env.veh_set[i].state[1])<env.veh_set[0].v_width and env.veh_set[0].state[0]>env.veh_set[i].state[0]:
                        collision = 2
                    else:
                        collision = 1

        else:
            break

        # print("t=",t)
        if t%N_update==0:
            veh_affordance=calc_affordance(env.veh_set,env.N_lane)
            TTC_traj_base=np.zeros([len(env.veh_set),N_base])
            TTC_traj_base1=10*np.ones([len(env.veh_set),N_base])
            safe_traj = [None]*len(env.veh_set)
            for i in range(0,len(env.veh_set)):
                if not env.veh_set[i].controlled:
                    for j in range(0,N_base):
                        TTC_traj_base[i][j] = check_collision(veh_affordance[i],traj_base[j][0:2*m],Ts,3)
                        fy = interpolate.interp1d(tt, traj_base[j][0:m]+env.veh_set[i].state[2]*tt+env.veh_set[i].state[0])
                        fx = interpolate.interp1d(tt, traj_base[j][m:2*m]+env.veh_set[i].state[1])
                        t_ts = np.arange(0,3+env.veh_set[i].ts,env.veh_set[i].ts)
                        Y_traj = fy(t_ts)
                        X_traj = fx(t_ts)
                        traj1=[Y_traj,X_traj]
                        L=env.veh_set[i].v_length
                        W=env.veh_set[i].v_width
                        for k in range(0,len(env.veh_set)):
                            # if k!=i and max([abs(env.veh_set[i].state[1]-env.veh_set[k].state[1])-3,abs(env.veh_set[i].state[0]-env.veh_set[k].state[0])-5])<0:
                            #     pdb.set_trace()
                            if k!=i and abs(env.veh_set[i].state[1]-env.veh_set[k].state[1])<5\
                            and (env.veh_set[i].state[0]<env.veh_set[k].state[0] or k!=0):

                                t2 = env.veh_set[k].t
                                length = len(env.veh_set[k].Y_traj)-t2



                                if k==0:
                                    traj2 = [env.veh_set[0].state[0]+t_ts*env.veh_set[0].state[2],np.ones(length)*env.veh_set[0].state[1]]
                                else:
                                    traj2 = [env.veh_set[k].Y_traj[t2:t2+length],env.veh_set[k].X_traj[t2:t2+length]]

                                TTC_traj_base1[i][j]=min(check_collision_simple([traj1[0][0:length],traj1[1][0:length]],traj2,L,W,env.veh_set[i].ts),TTC_traj_base1[i][j])
                    safe_traj[i]=[j for j, x in enumerate(TTC_traj_base1[i]) if x==max(TTC_traj_base1[i])]
                    # pdb.set_trace()
                    pred = env.pred_model(torch.tensor([veh_affordance[i,aff_idx]],dtype=torch.float32))[0][0].tolist()
                    # pdb.set_trace()
                    safe_pred = [pred[k]-offset[k] for k in safe_traj[i]]
                    idx = np.argmax(np.array(safe_pred))
                    traj_candidate = [j for j, x in enumerate(safe_traj[i]) if pred[x]>offset[x]]
                    # pdb.set_trace()
                    if len(traj_candidate)>0:
                        traj_choice = random.choice(traj_candidate)
                    else:
                        traj_choice = random.choice(safe_traj[i])

                    # traj_choice = safe_traj[i][idx]


                    # safe_traj[i]=[j for j, x in enumerate(TTC_traj_base[i]) if x==max(TTC_traj_base[i])]
                    # pdb.set_trace()

                    # pdb.set_trace()
                    env.veh_set[i].update_traj(traj_choice)


        u=env.step()
        if len(env.veh_set[0].x_pred)!=t+1:
            pdb.set_trace()
        input_rec[t]=u[0]
        for i in range(0,len(env.veh_set)):
            state_rec[i][t]=env.veh_set[i].state

        t=t+1
    # state_rec = np.array(state_rec)
    return state_rec,input_rec,collision,t

device = 'cuda' if torch.cuda.is_available() else 'cpu'
N_lane = 3
model = FCNet_new(op_dim=traj_base.shape[0]).to(device)
model = torch.load('traj_pred.pth')
model.eval()

def reactive_MPC_trial(n):
    print("test number ",n)
    h=Highway_env(N_HV=random.choice([2,3,4,5,6,7]),N_lane=N_lane,pred_model=model)
    state_rec,input_rec,collision,t=Highway_sim(h,20)
    if collision==1:
        print("collision")
        animate_scenario(h,state_rec,lm,traj_base,"failure"+str(n)+".mp4")
    elif collision ==2:
        print("rear hit")
        animate_scenario(h,state_rec,lm,traj_base,"rear_hit"+str(n)+".mp4")

    # pdb.set_trace()
    return t


def main():


    # device = 'cuda' if torch.cuda.is_available() else 'cpu'
    # N_lane = 3
    # model = FCNet_new(op_dim=traj_base.shape[0]).to(device)
    # model = torch.load('traj_pred.pth')
    # model.eval()
    # number of vehicles
    # veh_affordance=calc_affordance(h.veh_set,h.N_lane)
    # print(veh_affordance.shape)


## statistical study of collision rate

    pool = multiprocessing.Pool(processes=16)
    data = pool.map(reactive_MPC_trial, range(200))
    pool.close()
    pool.join()
    print('done')
    
    pdb.set_trace()

## Single simulation with animation

    # h=Highway_env(N_HV=5,N_lane=N_lane,pred_model=model)
    # state_rec,input_rec,collision,t=Highway_sim(h,10)
    # animate_scenario(h,state_rec,lm,traj_base,'MPC_movie1.mp4')

    # h=Highway_env(N_HV=5,N_lane=N_lane,pred_model=model)
    # state_rec,input_rec,collision,t=Highway_sim(h,10)
    # animate_scenario(h,state_rec,lm,traj_base,'MPC_movie2.mp4')

    # h=Highway_env(N_HV=5,N_lane=N_lane,pred_model=model)
    # state_rec,input_rec,collision,t=Highway_sim(h,10)
    # animate_scenario(h,state_rec,lm,traj_base,'MPC_movie3.mp4')

    # h=Highway_env(N_HV=5,N_lane=N_lane,pred_model=model)
    # state_rec,input_rec,collision,t=Highway_sim(h,10)
    # animate_scenario(h,state_rec,lm,traj_base,'MPC_movie4.mp4')




if __name__ == '__main__':
    main()
