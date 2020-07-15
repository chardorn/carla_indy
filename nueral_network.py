import numpy as np

def sigmoid(x):
    return 1.0/(1 + np.exp(-x))

def sigmoid_derivative(x):
    return x * (1-0 - x)


class NueralNetwork():

    #initialize variables: input, weights1, weights2, y, output
    def __init__(self, input, y):
        self.input =    x #Input Layer
        self.weights1 = np.random.rand(self.input.shape[1],4) #Hidden Layer 1
        self.weights2 = np.random.rand(4,1) #Hidden Layer 2
        self.y =        y   #Desired output
        self.output =   np.zeros(self.y.shape)  #Output Layer

    #Feedforward function
    #To calculate the value of 
    #y = sigmoid(Weight2 * sigmoid(Weight1 * x + b1) + b2)
    def feedforward(self):
        self.layer1 = sigmoid(np.dot(self.input, self.weights1))
        self.output = sigmoid(np.dot(self.layer1, self.weights2))

    #Loss function
    #We'll use a sum-of-squares loss function
    # = Sum from i = 1 to n of (ypredicted - yactual)^2

    #Gradient Descent
    #The derivative of the loss function dictates which direction is best to move
    #Use chain rule to get derivate of loss = 2(ypredicted - yactual) * z * (1 - z) * x where z = Wx + b

    def backprop(self):
        d_weights2 = np.dot(self.layer1.T, (2*(self.y - self.output) * sigmoid_derivative(self.output)))
        d_weights1 = np.dot(self.input.T, (np.dot(2*(self.y - self.output) * sigmoid_derivative(self.output), self.weights2.T) * sigmoid_derivative(self.layer1)))

        self.weights1 += d_weights1
        self.weights2 += d_weights2


if __name__ == "__main__":
    x = np.array([[0,0,1],
                  [0,1,1],
                  [1,0,1],
                  [1,1,1]])
    y = np.array([[0], [1], [1], [0]])
    n = NueralNetwork(x, y)
    
    for i in range(1500):
        n.feedforward()
        n.backprop()

    print(n.output)