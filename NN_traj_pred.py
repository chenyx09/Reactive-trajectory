import torch
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np
import torch.nn as nn
import torch.nn.functional as F
import pandas as pd
from scipy.io import loadmat

device = 'cuda' if torch.cuda.is_available() else 'cpu'


# Data Generation
class Model(nn.Module):

    def __init__(self, embedding_size, num_numerical_cols, output_size, layers, p=0.4):
        super().__init__()
        if embedding_size>0:
            self.all_embeddings = nn.ModuleList([nn.Embedding(ni, nf) for ni, nf in embedding_size])
            self.embedding_dropout = nn.Dropout(p)
        self.batch_norm_num = nn.BatchNorm1d(num_numerical_cols)

        all_layers = []
        if embedding_size>0:
            num_categorical_cols = sum((nf for ni, nf in embedding_size))
        else:
            num_categorical_cols = 0
        input_size = num_categorical_cols + num_numerical_cols

        for i in layers:
            all_layers.append(nn.Linear(input_size, i))
            all_layers.append(nn.ReLU(inplace=True))
            all_layers.append(nn.BatchNorm1d(i))
            all_layers.append(nn.Dropout(p))
            input_size = i

        all_layers.append(nn.Linear(layers[-1], output_size))

        self.layers = nn.Sequential(*all_layers)

    def forward(self, x_categorical, x_numerical):
        embeddings = []
        if hasattr(self,'all_embeddings'):
            for i,e in enumerate(self.all_embeddings):
                embeddings.append(e(x_categorical[:,i]))
            x = torch.cat(embeddings, 1)
            x = self.embedding_dropout(x)
        else:
            x=torch.tensor([], dtype=torch.float)

        x_numerical = self.batch_norm_num(x_numerical)
        x = torch.cat([x, x_numerical], 1)
        x = self.layers(x)
        return x

x = loadmat('data.mat')
data = x['training_data']
output = x['output']
weight = torch.zeros(output.shape[0],output.shape[1])
for i in range(0,output.shape[0]):
    for j in range(0,output.shape[1]):
        if output[i,j]==1:
            weight[i,j]=100
        elif output[i,j]==-1:
            weight[i,j]==1

total_records = data.shape[0]
M = output.shape[1]





numerical_data = torch.tensor(data, dtype=torch.float)

outputs = torch.tensor(output, dtype=torch.int64)





test_records = int(total_records * .2)

# categorical_train_data = categorical_data[:total_records-test_records]
# categorical_test_data = categorical_data[total_records-test_records:total_records]
# numerical_train_data = numerical_data[:total_records-test_records]
# numerical_test_data = numerical_data[total_records-test_records:total_records]
# train_outputs = outputs[:total_records-test_records]
# test_outputs = outputs[total_records-test_records:total_records]


model = Model(0, numerical_data.shape[1], M, [200,100,50,50], p=0.4)
# loss_function = nn.CrossEntropyLoss()
def loss_function(X,Y):
    return torch.norm(weight*(X-Y)**2)

optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
# def loss_function(X,Y):
#     return torch.mean((5000*Y+(1-Y))*torch.abs(X-Y))
#     # +torch.dot(torch.ones(Y.shape[0],dtype=torch.int64)-Y,torch.abs(X-Y))

epochs = 10000
aggregated_losses = []
empty_tensor = torch.tensor([], dtype=torch.int64)

for i in range(epochs):
    i += 1
    y_pred = model(empty_tensor, numerical_data)

    single_loss = loss_function(y_pred, outputs)
    aggregated_losses.append(single_loss)

    if i%40 == 0:
        print(y_pred)
        print(f'epoch: {i:3} loss: {single_loss.item():10.8f}')

    optimizer.zero_grad()
    single_loss.backward()
    optimizer.step()

print(f'epoch: {i:3} loss: {single_loss.item():10.10f}')

plt.plot(range(epochs), aggregated_losses)
plt.ylabel('Loss')
plt.xlabel('epoch');
