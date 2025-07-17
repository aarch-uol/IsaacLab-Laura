import torch
from torch.utils.data import Dataset, DataLoader, random_split
import torch.nn as nn


class JointSafetyDataset(Dataset):
    def __init__(self, data):
        self.inputs = torch.tensor([d[0] for d in data], dtype=torch.float32)
        self.labels = torch.tensor([d[1] for d in data], dtype=torch.float32).unsqueeze(1)

    def __len__(self):
        return len(self.inputs)
    
    def __getitem__(self, idx):
        return self.inputs[idx], self.labels[idx]
    

class JointSafetyClassifier(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(7, 128),
            nn.ReLU(),
            nn.Linear(128, 1),
            nn.Sigmoid()
        )

    def forward(self, x):
        return self.net(x)

    def isSafe(self, x):
        y = self.forward(x)
        return y > 0.5


def load_data(file):
    data = []
    with open(file, 'r') as file:
        for line in file.readlines():
            line = line.strip()
            vector_str, label_str = line.rsplit(']', 1)
            joint_positions = eval(vector_str + ']')
            label = int(label_str.strip())
            data.append((joint_positions, label))      

    return data      


def calculate_accuracy(loader, model):
    model.eval()
    correct = 0
    total = 0
    with torch.no_grad():
        for inputs, labels in loader:
            outputs = model(inputs)
            predicted = (outputs >= 0.5).float()  # Threshold sigmoid output at 0.5
            correct += (predicted == labels).sum().item()
            total += labels.size(0)
    return correct / total


# data = load_data("./docs/training_data/training_data.txt")
# dataset = JointSafetyDataset(data)

# # Split dataset into 80% train, 20% test
# train_size = int(0.8 * len(dataset))
# test_size = len(dataset) - train_size
# train_dataset, test_dataset = random_split(dataset, [train_size, test_size])

# train_loader = DataLoader(train_dataset, batch_size=64, shuffle=True)
# test_loader = DataLoader(test_dataset, batch_size=64, shuffle=False)

# model = JointSafetyClassifier()
# criterion = nn.BCELoss()
# optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)

# for epoch in range(40):
#     model.train()
#     for batch_x, batch_y in train_loader:
#         pred = model(batch_x)
#         loss = criterion(pred, batch_y)
#         optimizer.zero_grad()
#         loss.backward()
#         optimizer.step()

#     train_acc = calculate_accuracy(train_loader, model)
#     test_acc = calculate_accuracy(test_loader, model)
#     print(f"Epoch {epoch+1}, Loss: {loss.item():.4f}, Train Acc: {train_acc:.4f}, Test Acc: {test_acc:.4f}")

# torch.save(model.state_dict(), './docs/training_data/JointSafetyClassifier.pth')
