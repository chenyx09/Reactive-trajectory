from __future__ import print_function
import torch
import argparse
import torchvision.transforms as transforms
import matplotlib
# matplotlib.use("Agg")
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


device = 'cuda' if torch.cuda.is_available() else 'cpu'


T = 3.
Ts = 0.5
m = int(T/Ts+1)

idx = [2,3,4,5,8,9,10,11,12,13,14,15,16,17,18,19,24,25,26,27,28]
lm = [0,3.6,7.2,10.8,14.4,18,21.6]
x = loadmat('offset.mat')
offset = x['offset'][0]
x = loadmat('SVM_res.mat')
w_val=x['w_val']

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


def animate_scenario(veh_ID,t_min,t_max,frames,lm,aff,traj_base,model):
    fig = plt.figure()
    plt.xlim(0, 21.6)
    ax = fig.add_subplot(111)
    plt.grid()
    nframe = int((t_max-t_min)/100)
    t_frames=np.zeros(frames.shape[0])
    for i in range(0,frames.shape[0]):
        t_frames[i]=frames[i][0][0]

    plotted_veh_ID = []
    near_veh = []
    frame = frames[np.where(t_frames == t_min)][0]
    plot_veh_frame = frame[np.where(frame[:, 1] == veh_ID)][0]
    ego_veh = plt.Rectangle((plot_veh_frame[2] - plot_veh_frame[6] / 2, plot_veh_frame[3] - plot_veh_frame[5] / 2),
                            plot_veh_frame[6], plot_veh_frame[5], fc='r', zorder=0)
    # def init():
    #     for j in range(0, len(lm)):
    #         plt.plot([lm[j], lm[j]], [-10, 1000], 'go--', linewidth=2)
    #
    #     ax.add_patch(ego_veh)
    #     return ego_veh

    def animate(j,plotted_veh_ID,near_veh,ego_veh):
        t = t_min+j*100
        # print(t)
        t_idx = np.where(t_frames==t)[0][0]
        frame = frames[t_idx]
        plot_veh_frame = frame[np.where(frame[:,1]==veh_ID)][0]
        ego_y = plot_veh_frame[3]
        # print(ego_y)
        near_veh_frame = frame[np.where((frame[:,3]>=ego_y-8) & (frame[:,3]<=ego_y+51) & (frame[:,1]!=veh_ID))]

        ego_veh.set_xy([plot_veh_frame[2]-plot_veh_frame[6]/2, plot_veh_frame[3]-plot_veh_frame[5]/2])
        for j in range(0,near_veh_frame.shape[0]):
            if near_veh_frame[j][1] not in plotted_veh_ID:
                near_veh.append(plt.Rectangle((near_veh_frame[j][2]-near_veh_frame[j][6]/2, near_veh_frame[j][3]-near_veh_frame[j][5]/2), near_veh_frame[j][6], near_veh_frame[j][5], fc='b', zorder=0))
                plotted_veh_ID.append(near_veh_frame[j][1])
                # print(plotted_veh_ID)
            else:
                idx1 = plotted_veh_ID.index(near_veh_frame[j][1])
                # print(near_veh_frame[j][1])
                # print(len(near_veh))
                near_veh[idx1].set_xy([near_veh_frame[j][2]-near_veh_frame[j][6]/2, near_veh_frame[j][3]-near_veh_frame[j][5]/2])

        ax.clear()
        ax.add_patch(ego_veh)

        affordance = aff[t_idx][np.where(aff[t_idx][:,0]==veh_ID)][0]
        affordance = affordance[1:]
        # print(affordance)
        pred = model(torch.tensor([affordance],dtype=torch.float32))[0][0].tolist()
        feature = model(torch.tensor([affordance],dtype=torch.float32))[1][0].tolist()
        feature.insert(0,1.)
        pred1 = np.matmul(w_val,feature)

        v0 = plot_veh_frame[7]
        x0 = plot_veh_frame[2]
        y0 = plot_veh_frame[3]
        y_traj0 = 0*np.linspace(0,T,m)
        # print(pred)
        # print(max(pred))
        # if max(pred)>0.5:
        for j in range(0,len(pred)):
            # if pred[j]>offset[j]:            # result using bloating
            if pred1[j]>0:                     # result using SVM
                y_traj = y_traj0 + traj_base[j][0:m]+y0
                x_traj = traj_base[j][m:]+x0
                plt.plot(x_traj,y_traj,'r--',linewidth=1)
        for k in range(0,len(near_veh)):
            # print(len(near_veh))
            if plotted_veh_ID[k] in near_veh_frame[:,1]:
                ax.add_patch(near_veh[k])
        for j in range(0, len(lm)):
            plt.plot([lm[j], lm[j]], [-10, 1000], 'go--', linewidth=2)
        # else:
        #     max_idx = pred.index(max(pred))
        #     print(max_idx)
        #     y_traj = y_traj0 + traj_base[max_idx][0:m] + y0
        #     x_traj = traj_base[max_idx][m:] + x0
        #     plt.plot(x_traj, y_traj, 'r--', linewidth=1)

        ax.axis('equal')
        ax.set_xlim(0, 21.6)
        ax.set_ylim(ego_y-7, ego_y+50)


        # return near_veh
        # print(len(ax.patches))
        # print(len(plotted_veh_ID))
        return plotted_veh_ID, near_veh, ego_veh
    anim = animation.FuncAnimation(fig, animate, fargs=(plotted_veh_ID,near_veh,ego_veh,),
                                   frames=nframe,
                                   interval=100,
                                   blit=False, repeat=False)
    plt.show()
    # Writer = animation.writers['ffmpeg']
    # writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)
    # anim_name = 'anim'+str(veh_ID)+'.mp4'
    # anim.save(anim_name,writer=writer)
def main():

    data = loadmat('scenarios.mat')
    aff = data['affordance_set1'][0]
    traj_base = data['traj_base']
    frames = data['frames'][0]
    veh_traj = data['veh_traj_set'][0]
    model = FCNet_new(op_dim=traj_base.shape[0]).to(device)
    model = torch.load('traj_pred.pth')
    model.eval()
    duration = np.zeros(veh_traj.shape[0])
    for i in range(0,veh_traj.shape[0]):
        duration[i]=veh_traj[i][1][0][1]
    order=np.argsort(-duration)
    for i in range(0,10):
        idx1 = order[i]
        veh_ID = veh_traj[idx1][0][0][0]
        t_min = veh_traj[idx1][1][0][0]
        t_max = veh_traj[idx1][1][0][0] + veh_traj[idx1][1][0][1]
        animate_scenario(veh_ID,t_min,t_max,frames,lm,aff,traj_base,model)




if __name__ == '__main__':
    main()
