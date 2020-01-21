from __future__ import print_function
import torch
import argparse
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
from scipy.io import loadmat
from torch.utils import data

device = 'cuda' if torch.cuda.is_available() else 'cpu'



class FCNet(nn.Module):
    def __init__(self, hidden_1=30, hidden_2=30 ,op_dim=30, input_dim=21):
        super().__init__()
        self.fc1 = nn.Linear(input_dim, hidden_1)
        self.fc2 = nn.Linear(hidden_2, op_dim)
        self.bn1 = nn.BatchNorm1d(hidden_1)

    def forward(self, x):
        output = F.relu(self.bn1(self.fc1(x)))
        output = self.fc2(output)
        output = F.log_softmax(output, dim=1)
        return output

def loss_function(X,Y, weight):
    return torch.norm(weight*(X-Y)**2)

def train(args, model, device, train_loader, optimizer, epoch, weight):
    for batch_idx, (input, target) in enumerate(train_loader):
        input, target = input.to(device), target.to(device)
        optimizer.zero_grad()
        output = model(input)
        loss = loss_function(target, output, weight)
        loss.backward()
        optimizer.step()
        if batch_idx % args.log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch, batch_idx * len(input), len(train_loader.dataset),
                100. * batch_idx / len(train_loader), loss.item()))



def main():
    parser = argparse.ArgumentParser(description='PyTorch MNIST Example')

    parser.add_argument('--log-interval', type=int, default=10, metavar='N',
                        help='how many batches to wait before logging training status')
    args = parser.parse_args()

    x = loadmat('data.mat')
    affordance_data = x['training_data'][:100]
    output = x['output'][:100]
    weight = torch.zeros(output.shape[0],output.shape[1])
    for i in range(0,output.shape[0]):
        for j in range(0,output.shape[1]):
            if output[i,j]==1:
                weight[i,j]=100
            elif output[i,j]==-1:
                weight[i,j]==1

    affordance_data = torch.Tensor(affordance_data)
    output = torch.Tensor(output)
    output_shape = output.shape[1]

    model = FCNet(op_dim=output_shape).to(device)
    affordance_dataset = data.TensorDataset(affordance_data, output)
    affordance_dataloader = data.DataLoader(affordance_dataset,
                                            batch_size = 128)

    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    epoch = 0

    train(args=args, model=model, device=device, train_loader=affordance_dataloader,
          optimizer=optimizer, epoch=epoch, weight=weight)




if __name__ == '__main__':
    main()
