import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import torchvision
import numpy as np
import h5py


class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        # N is batch size, D_in is input dimension, D_out is output dimension (dim=2 means 2-class classification)
        D_in, H_1, H_2, D_out = 4800, 2000, 100, 2
        self.linear1 = nn.Linear(D_in, H_1)
        self.linear2 = nn.Linear(H_1, H_2)
        self.linear3 = nn.Linear(H_2, D_out)

    def forward(self, x):
        h1_relu = F.relu(self.linear1(x))
        h2_relu = F.relu(self.linear2(h1_relu))
        y_pred = self.linear3(h2_relu)
        return y_pred




### Preparations
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print(device)


### Load datasets and convert to PyTorch Tensor(using torch.tensor())
# load data from h5 file
file_name = 'test_imi_data_YuMi_training_dataset.h5'
f = h5py.File(file_name, 'r')
x_train = f['x_train'][:].transpose()
y_train = f['y_train'][:].transpose()
x_test = f['x_test'][:].transpose()
y_test = f['y_test'][:].transpose()
# convert numpy.ndarray to torch.tensor()
x_train, y_train, x_test, y_test = map(
    torch.tensor, (x_train, y_train, x_test, y_test)
)
# construct Dataset and DataLoader to automatically handle minibatches
bs = 64
train_ds = TensorDataset(x_train, y_train)
train_dl = DataLoader(train_ds, batch_size=bs, shuffle=True)
test_ds = TensorDataset(x_test, y_test)
test_dl = DataLoader(test_ds, batch_size=bs*2)


### Build networks
# nn model
print('Input dim: ' + str(x_train.shape[1]))
model = Net()
model.to(device) # for GPU computing

# loss function
criterion = nn.CrossEntropyLoss()
# optimization algorithms
learning_rate = 1e-4
optimizer = optim.SGD(model.parameters(), lr=learning_rate)#, momentum=0.9) #torch.optim.Adam(model.parameters(), lr=learning_rate)

### Training
epochs = 1000
loss_history = []
for epoch in range(epochs):
    running_loss = 0.0
    print('Epoch ' + str(epoch+1) + ' / ' + str(epochs))
    for i, data in enumerate(train_dl, 0):
        #print('batch ' + str(i+1))
        # get the inputs and labels
        xb, yb = data[0].to(device, dtype=torch.float), data[1].to(device, dtype=torch.long) # send to GPU at every step

        # Zero the gradients
        optimizer.zero_grad()  # zero the gradients before running the backward pass

        # Forward pass
        y_pred = model(xb)
        #import pdb
        #pdb.set_trace()
        loss = criterion(y_pred, yb.squeeze())  # .squeeze() to make yb.shape from [bs, 1] to [bs]
        loss_history.append(loss.item())
        # Backward pass
        loss.backward()
        # update
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if i % 10 == 9: # print every 2000 mini-batches
            print('[%d, %5d] loss: %.3f' % (epoch + 1, i + 1, running_loss / 10))
            running_loss = 0.0

print('Finished Training')

import pdb
pdb.set_trace()

### Save trained model
model_path = './trained_model.pth'
torch.save(model.state_dict(), model_path)


### Test the network on test data
net = Net()
net.load_state_dict(torch.load(model_path))
net.to(device)
correct = 0
total = 0
'''
with torch.no_grad():
    for data in test_dl:
        # get inputs and labels
        xb, yb = data[0].to(device, dtype=torch.float), data[1].to(device, dtype=torch.long)
        # evaluate
        outputs = net(xb)
        _, predicted = torch.softmax(outputs.data, 1)
        total += yb.size(0)
        correct += (predicted == yb).sum().item()

print('Accuracy of the network: %d %%' % (100 * correct / total))
'''

# simpler version
acc_sum = (net(x_test.to(device, dtype=torch.float)).argmax(dim=1) == y_test.to(device, dtype=torch.long).squeeze() ).sum()
print("test accuracy: %f" % (torch.true_divide(acc_sum, y_test.shape[0])))

