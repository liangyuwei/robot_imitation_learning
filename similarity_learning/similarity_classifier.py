import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import TensorDataset, DataLoader
import torchvision
import numpy as np
import h5py

# for deepcopy
import copy

# for display
import matplotlib.pyplot as plt


def show_plot(iteration, loss):
    # Plot the loss history
    plt.plot(iteration, loss)
    plt.title('Loss History')
    plt.grid()
    plt.xlabel('Iterations')
    plt.ylabel('Loss')
    plt.show()

def calc_rate(dist_results, ground_truths, threshold):
    # initialize
    TP = 0
    FP = 0
    FN = 0
    TN = 0
    num_data = dist_results.shape[0]
    # classify according to threshold
    for i in range(num_data):
        if(dist_results[i] < threshold): # for cosine distance
        #if (dist_results[i] > threshold): # for euclidean distance
            # distance large enough to be classified as dissimilar (negative class)
            if (int(ground_truths[i]) == 1):
                # actually positive class
                FN += 1
            else:
                # actually negative class
                TN += 1
        else:
            # distance smaller than or equal to threshold, classified as similar (positive class)
            if (int(ground_truths[i]) == 1):
                # actually positive class
                TP += 1
            else:
                # actually negative class
                FP += 1
    # calculate evaluation metrics
    if TP+FP == 0:
        Precision = 0
    else:
        Precision = float(TP) / float(TP + FP)
    Recall = float(TP) / float(TP + FN)
    Accuracy = float(TP+TN) / float(num_data) # accurately classified
    TPR = float(TP) / float(TP + FN)
    TNR = float(TN) / float(TN + FP)
    FNR = float(FN) / float(TP+FN)
    FPR = float(FP) / float(TN + FP)
    if Precision+Recall == 0:
        F1_score = 0
    else:
        F1_score = 2 * Precision * Recall / (Precision+Recall)
    return Accuracy, Precision, Recall, F1_score, TPR, TNR, FNR, FPR


class ContrastiveLoss(torch.nn.Module):
    """
    Contrastive loss function.
    Based on: http://yann.lecun.com/exdb/publis/pdf/hadsell-chopra-lecun-06.pdf
    """
    def __init__(self, margin=2.0):
        super(ContrastiveLoss, self).__init__()
        self.margin = margin

    def forward(self, output1, output2, label):

        # 1 - Euclidean_distance (for similar pair, euclidean_distance better be small)
        euclidean_distance = F.pairwise_distance(output1, output2)
        # Contrastive loss
        loss_contrastive = torch.mean((label) * torch.pow(euclidean_distance, 2) + \
        (1 - label) * torch.pow(torch.clamp(self.margin - euclidean_distance, min=0.0), 2))

        # 2 - Cosine distance (for similar pair, cosine_distance better be large, closer to 1)
        #cosine_distance = F.cosine_similarity(output1, output2) # only between -1 and 1, margin.. be 0?
        # Contrastive loss
        #c_margin = -0.5
        #loss_contrastive = torch.mean((label) * torch.pow((cosine_distance-1), 2) + \
        #                    (1 - label) * torch.pow(F.relu(cosine_distance - c_margin), 2))
        # hope that cosine_distance to be negative when dissimilar, lower than margin, which is negative

        return loss_contrastive


class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        # N is batch size, D_in is input dimension, D_out is output dimension (dim=2 means 2-class classification)
        D_in, H_1, H_2, D_out = 2400, 1200, 500, 100 #4800, 2400, 1000, 100
        self.linear1 = nn.Linear(D_in, H_1)
        self.linear2 = nn.Linear(H_1, H_2)
        self.linear4 = nn.Linear(H_3, D_out)
        #self.dropout = nn.Dropout(p=0.5)

    def forward_once(self, x):
        h1_drop = F.relu(self.linear1(x))
        h2_drop = F.relu(self.linear2(h1_drop))
        h3_drop = F.relu(self.linear3(h2_drop))
        y_pred = self.linear4(h3_drop)
        return y_pred

    def forward(self, input1, input2):
        output1 = self.forward_once(input1)
        output2 = self.forward_once(input2)
        return output1, output2

class Net_for_output(nn.Module):

    def __init__(self):
        super(Net_for_output, self).__init__()
        # N is batch size, D_in is input dimension, D_out is output dimension (dim=2 means 2-class classification)
        D_in, H_1, H_2, D_out = 2400, 1200, 500, 100 #4800, 2400, 1000, 100
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
file_name = 'test_imi_data_YuMi_training_dataset-50datapoints.h5'
f = h5py.File(file_name, 'r')
x_train = f['x_train'][:].transpose()
y_train = f['y_train'][:].transpose()
x_test = f['x_test'][:].transpose()
y_test = f['y_test'][:].transpose()
# split the sample pairs for ease of feedforwarding network
#half_size = round(x_train.shape[1]/2)
x_train1 = copy.deepcopy(x_train[:, 0:2400])#(x_train[:, 0:4800])
x_train2 = copy.deepcopy(x_train[:, 2400:])#(x_train[:, 4800:])
#half_size = round(x_test.shape[1]/2)
x_test1 = copy.deepcopy(x_test[:, 0:2400])#(x_test[:, 0:4800])
x_test2 = copy.deepcopy(x_test[:, 2400:])#(x_test[:, 4800:])
# convert numpy.ndarray to torch.tensor()
x_train1, x_train2, y_train, x_test1, x_test2, y_test = map(
    torch.tensor, (x_train1, x_train2, y_train, x_test1, x_test2, y_test)
)
# construct Dataset and DataLoader to automatically handle minibatches
bs = 256
train_ds = TensorDataset(x_train1, x_train2, y_train)
train_dl = DataLoader(train_ds, batch_size=bs, shuffle=True)
train_eval_dl = DataLoader(train_ds, batch_size=bs*2)
test_ds = TensorDataset(x_test1, x_test2, y_test)
test_dl = DataLoader(test_ds, batch_size=bs*2)


### Build networks
# nn model
print('Input dim: ' + str(x_train.shape[1]))
model = Net()
model.to(device) # for GPU computing

# loss function
criterion = ContrastiveLoss() #nn.CrossEntropyLoss()
# optimization algorithms
learning_rate = 1e-4
#optimizer = optim.SGD(model.parameters(), lr=learning_rate)#, momentum=0.9) #torch.optim.Adam(model.parameters(), lr=learning_rate)
optimizer = optim.Adam(model.parameters(), lr=learning_rate)

### Training
epochs = 500
counter = []
loss_history = []
iteration_number = 0
for epoch in range(epochs):
    running_loss = 0.0
    print('Epoch ' + str(epoch+1) + ' / ' + str(epochs))
    for i, data in enumerate(train_dl, 0):
        #print('batch ' + str(i+1))
        # get the inputs and labels
        #print('Get the inputs and labels.')
        xb1, xb2, yb = data[0].to(device, dtype=torch.float), data[1].to(device, dtype=torch.float), data[2].to(device, dtype=torch.long) # send to GPU at every step
    
        # Zero the gradients
        #print('Zero the gradients')
        optimizer.zero_grad()  # zero the gradients before running the backward pass

        # Forward pass
        #print('Forward pass..')
        output1, output2 = model(xb1, xb2)
        #import pdb
        #pdb.set_trace()
        #print('Calculate the loss..')
        loss = criterion(output1, output2, yb.squeeze())  # .squeeze() to make yb.shape from [bs, 1] to [bs]

        # Backward pass
        #print('Backward pass..')
        loss.backward()
        # update
        #print('Update parameters..')
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if i % 10 == 0: # print every 2000 mini-batches
            # print loss
            print('[%d, %5d] loss: %.3f' % (epoch + 1, i + 1, running_loss / 10))
            running_loss = 0.0
            # record history for later plot
            loss_history.append(loss.item())
            iteration_number += 10
            counter.append(iteration_number)

print('Finished Training')

show_plot(counter, loss_history)


#import pdb
#pdb.set_trace()


### Save trained model
model_path = './trained_model_adam_euclidean_epoch500_bs256_group_split_dataset-50p.pth'
torch.save(model.state_dict(), model_path)


#import pdb
#pdb.set_trace()


### Convert .pth model to Torch Script .pt file
'''
# load the trained .pth model
tmp_model_path = '/home/liangyuwei/dataset_backup/groups_split_current_20200517/trained_model_adam_euclidean_epoch500_bs128_group_split_dataset.pth'#model_path
tmp_net = Net_for_output()
print('Load trained model..')
tmp_net.load_state_dict(torch.load(tmp_model_path, map_location=torch.device('cpu')))
#tmp_net.to(device)
# set a random example
example = torch.rand(1, 4800)
# generate ScriptModule via tracing (or annotation...)
traced_script_module = torch.jit.trace(tmp_net, example) # generate a torch.jit.ScriptModule via tracing
# can now be evaluated
print('Evaluate the ScriptModule:')
output = traced_script_module(torch.ones(1, 4800))
import pdb
pdb.set_trace()
# serialize ScriptModule to a file
traced_script_module.save("traced_model_adam_euclidean_epoch500_bs128_group_split_dataset.pt")

import pdb
pdb.set_trace()
'''


### Test the network on test data
net = Net()
print('Load trained model..')
net.load_state_dict(torch.load(model_path))
net.to(device)
dist_result_all = np.array([])
labels_all = np.array([]) # for test set
train_dist_result_all = np.array([])
train_labels_all = np.array([]) # for train set
print('Evaluating test set..')
with torch.no_grad():
    # on test set
    for data in test_dl:
        # get inputs and labels
        xb1, xb2, yb = data[0].to(device, dtype=torch.float), data[1].to(device, dtype=torch.float), data[2].to(device, dtype=torch.long)
        # evaluate
        output1, output2 = net(xb1, xb2) # output features N x 100
        #dist_result = F.pairwise_distance(output1, output2) # torch.Size([N]) to array?
        dist_result = F.cosine_similarity(output1, output2) # torch.Size([N]) to array?
        # store the result
        dist_result_cpu = dist_result.cpu() # if using gpu, should copy the tensor to host memory first
        dist_result_all = np.concatenate((dist_result_all, dist_result_cpu.numpy())) # .concatenate(), deep copy
        yb_cpu = yb.cpu() # if using gpu, should copy the tensor to host memeory first
        labels_all = np.concatenate((labels_all, yb_cpu.numpy().squeeze())) # .concatenate(), deep copy

    # on train set
    for data in train_eval_dl:
        # get inputs and labels
        xb1, xb2, yb = data[0].to(device, dtype=torch.float), data[1].to(device, dtype=torch.float), data[2].to(device, dtype=torch.long)
        # evaluate
        output1, output2 = net(xb1, xb2) # output features N x 100
        #dist_result = F.pairwise_distance(output1, output2) # torch.Size([N]) to array?
        dist_result = F.cosine_similarity(output1, output2) # torch.Size([N]) to array?
        # store the result
        dist_result_cpu = dist_result.cpu() # if using gpu, should copy the tensor to host memory first
        train_dist_result_all = np.concatenate((train_dist_result_all, dist_result_cpu.numpy())) # .concatenate(), deep copy
        yb_cpu = yb.cpu() # if using gpu, should copy the tensor to host memeory first
        train_labels_all = np.concatenate((train_labels_all, yb_cpu.numpy().squeeze())) # .concatenate(), deep copy


# compute evaluation metrics by iteration over different thresholds
# for test set
print('Test set: Set up threshold candidates..')
thres_candi = sorted(dist_result_all)
num_thres_candi = len(thres_candi)
Accuracy_array = np.zeros(num_thres_candi)
Precision_array = np.zeros(num_thres_candi)
Recall_array = np.zeros(num_thres_candi)
F1_score_array = np.zeros(num_thres_candi)
TPR_array = np.zeros(num_thres_candi)
TNR_array = np.zeros(num_thres_candi)
FNR_array = np.zeros(num_thres_candi)
FPR_array = np.zeros(num_thres_candi)
print('Test set: Calculate evaluation metrics..')
for th in range(num_thres_candi):
    Accuracy, Precision, Recall, F1_score, TPR, TNR, FNR, FPR = calc_rate(dist_result_all, labels_all, thres_candi[th]) # threshold from minimum distance to maximum distance
    Accuracy_array[th] = Accuracy
    Precision_array[th] = Precision
    Recall_array[th] = Recall
    F1_score_array[th] = F1_score
    TPR_array[th] = TPR
    TNR_array[th] = TNR
    FNR_array[th] = FNR
    FPR_array[th] = FPR

# for train set
print('Train set: Set up threshold candidates..')
train_thres_candi = sorted(train_dist_result_all)
train_num_thres_candi = len(train_thres_candi)
train_Accuracy_array = np.zeros(train_num_thres_candi)
train_Precision_array = np.zeros(train_num_thres_candi)
train_Recall_array = np.zeros(train_num_thres_candi)
train_F1_score_array = np.zeros(train_num_thres_candi)
train_TPR_array = np.zeros(train_num_thres_candi)
train_TNR_array = np.zeros(train_num_thres_candi)
train_FNR_array = np.zeros(train_num_thres_candi)
train_FPR_array = np.zeros(train_num_thres_candi)
print('Train set: Calculate evaluation metrics..')
for th in range(train_num_thres_candi):
    Accuracy, Precision, Recall, F1_score, TPR, TNR, FNR, FPR = calc_rate(train_dist_result_all, train_labels_all, train_thres_candi[th]) # threshold from minimum distance to maximum distance
    train_Accuracy_array[th] = Accuracy
    train_Precision_array[th] = Precision
    train_Recall_array[th] = Recall
    train_F1_score_array[th] = F1_score
    train_TPR_array[th] = TPR
    train_TNR_array[th] = TNR
    train_FNR_array[th] = FNR
    train_FPR_array[th] = FPR

print('done.')


# post-processing and display
# test set
AUC = abs(np.trapz(TPR_array, FPR_array)) # sometimes value is negative due to diff negative..
threshold = np.argmin(abs(FNR_array - FPR_array))
EER = (FNR_array[threshold]+FPR_array[threshold])/2 # theoretically, EER is when FNR == FPR, optimizing the trade-off
# train set
train_AUC = abs(np.trapz(train_TPR_array, train_FPR_array))
train_threshold = np.argmin(abs(train_FNR_array - train_FPR_array))
train_EER = (train_FNR_array[train_threshold]+train_FPR_array[train_threshold])/2 # theoretically, EER is when FNR == FPR, optimizing the trade-off

print("Result: ")
print("[Test set statistics]: ")
print("AUC: %f, Best threshold: %f, EER: %f " % (AUC, thres_candi[threshold], EER))
print("Accuracy at EER: %f, Precision at EER: %f, Recall at EER: %f, TPR: %f, TNR: %f \n P.S: approximately at EER since the samples are discrete" % \
      (Accuracy_array[threshold], Precision_array[threshold], Recall_array[threshold], TPR_array[threshold], TNR_array[threshold]) )
print("[Train set statistics]: ")
print("AUC: %f, Best threshold: %f, EER: %f " % (train_AUC, train_thres_candi[train_threshold], train_EER))
print("Accuracy at EER: %f, Precision at EER: %f, Recall at EER: %f, TPR: %f, TNR: %f \n P.S: approximately at EER since the samples are discrete" % \
      (train_Accuracy_array[train_threshold], train_Precision_array[train_threshold], train_Recall_array[train_threshold], train_TPR_array[train_threshold], train_TNR_array[train_threshold]) )


plt.plot(FPR_array, TPR_array) # for test set
plt.plot(train_FPR_array, train_TPR_array, c='red') # for train set
plt.plot(np.array([0.0, 1.0]), np.array([0.0, 1.0]), c='green') # random guess...
plt.xlim(0.0, 1.0)
plt.ylim(0.0, 1.0)
plt.title('ROC curve')
plt.xlabel('FPR_array')
plt.ylabel('TPR_array')
plt.grid()
plt.show()


