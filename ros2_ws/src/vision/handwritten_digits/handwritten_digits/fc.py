#
# MOBILE ROBOTS - FI-UNAM, 2026-2
# TRAINING A NEURAL NETWORK
#
# Instructions:
# Complete the code to train a fully connected neural network for
# handwritten digit recognition.
#
import cv2
import sys
import random
import numpy
import os

NAME = "Marco Ruben Guerrero Nieva"

class FCNeuralNetwork(object):
    def __init__(self, layers, weights=None, biases=None):
        #
        # The list 'layers' indicates the number of neurons in each layer.
        # Remember that the first layer indicates the input dimension. 
        # All weights and biases are initialized with random values. In each layer (except the first one)
        # we have a matrix of weights where row j contains all the weights of the j-th neuron in that layer,
        # and a vector of biases.
        #
        self.biases =[numpy.random.randn(y,1) for y in layers[1:]] if biases == None else biases
        self.weights=[numpy.random.randn(y,x) for x,y in zip(layers[:-1],layers[1:])] if weights==None else weights
        
    def feedforward(self, x):
        y = []
        #
        # Output of every layer for the input x.
        # y[0]   = x            (layer 0, the input itself)
        # y[i+1] = sigmoid(W[i] @ y[i] + B[i])
        #
        y.append(x)
        for w, b in zip(self.weights, self.biases):
            u = numpy.dot(w, y[-1]) + b          # pre-activation: z = W a + b
            a = 1.0 / (1.0 + numpy.exp(-u))      # sigmoid: a = 1/(1+exp(-z))
            y.append(a)
        
        return y

    def backpropagate(self, x, t):
        y = self.feedforward(x)
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        #
        # Gradients of J = (1/2) * ||y[-1] - t||^2 for a single example (x, t).
        #
        # Output layer L: delta_L = (a_L - t) * a_L * (1 - a_L)
        # (the derivative of the cost with respect to z_L; the factor (1/2)
        # cancels the 2 of the quadratic cost).
        #
        delta = (y[-1] - t) * y[-1] * (1.0 - y[-1])
        nabla_b[-1] = delta
        nabla_w[-1] = numpy.dot(delta, y[-2].T)
        #
        # Hidden layers, backwards: delta_l = (W_{l+1}^T delta_{l+1}) * a_l * (1 - a_l)
        # i = 2 -> layer L-1, i = 3 -> layer L-2, ...
        #
        for i in range(2, len(y)):
            delta = numpy.dot(self.weights[-i+1].T, delta) * y[-i] * (1.0 - y[-i])
            nabla_b[-i] = delta
            nabla_w[-i] = numpy.dot(delta, y[-i-1].T)
        
        return nabla_w, nabla_b

    def update_with_batch(self, batch, eta):
        #
        # This function exectutes gradient descend for the subset of examples
        # given by 'batch' with learning rate 'eta'
        # 'batch' is a list of training examples [(x,t), ..., (x,t)]
        # Function returns the magnitude of calculated gradient 
        #
        batch_nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        batch_nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        M = len(batch)
        mag_nabla = 0
        for x,t in batch:
            nabla_w, nabla_b = self.backpropagate(x,t)
            for j in range(len(nabla_w)):
                batch_nabla_w[j] += nabla_w[j]/M
                batch_nabla_b[j] += nabla_b[j]/M
        for j in range(len(nabla_b)):
            self.weights[j] = self.weights[j] - eta*batch_nabla_w[j]
            self.biases[j]  = self.biases[j]  - eta*batch_nabla_b[j]
            mag_nabla += numpy.linalg.norm(batch_nabla_w[j]) + numpy.linalg.norm(batch_nabla_b[j])
        return mag_nabla

    def train_by_SGD(self, training_x, training_t, epochs, batch_size, eta):
        #
        # This function implements the Stochastic Gradient Descend
        #
        training_data = list(zip(training_x, training_t))
        for j in range(epochs):
            random.shuffle(training_data)
            batches = [training_data[k:k+batch_size] for k in range(0,len(training_data), batch_size)]
            for batch in batches:
                nabla_magnitude = self.update_with_batch(batch, eta)
                sys.stdout.write("\rGradient magnitude: %f            " % (nabla_magnitude))
                sys.stdout.flush()
            print("Epoch: " + str(j))
    #
    ### END OF CLASS
    #


def load_dataset(folder):
    print("Loading data set from " + folder)
    training_x, training_t, testing_x, testing_t = [],[],[],[]
    labels = [[1,0,0,0,0,0,0,0,0,0], [0,1,0,0,0,0,0,0,0,0], [0,0,1,0,0,0,0,0,0,0],
              [0,0,0,1,0,0,0,0,0,0], [0,0,0,0,1,0,0,0,0,0], [0,0,0,0,0,1,0,0,0,0],
              [0,0,0,0,0,0,1,0,0,0], [0,0,0,0,0,0,0,1,0,0], [0,0,0,0,0,0,0,0,1,0],
              [0,0,0,0,0,0,0,0,0,1]]
    # labels = [[0,0,0,0], [0,0,0,1], [0,0,1,0], [0,0,1,1], [0,1,0,0],
    #           [0,1,0,1], [0,1,1,0], [0,1,1,1], [1,0,0,0], [1,0,0,1]]
    for i in range(10):
        f_data = [c/255.0 for c in open(os.path.join(folder, "data" + str(i)), "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784,1]) for j in range(1000)]
        label  = numpy.asarray(labels[i]).reshape([10,1])
        # label  = numpy.asarray(labels[i]).reshape([4,1])
        training_x += images[0:len(images)//2]
        training_t += [label for j in range(len(images)//2)]
        testing_x  += images[len(images)//2:len(images)]
        testing_t  += [label for j in range(len(images)//2)]
    return training_x, training_t, testing_x, testing_t

def main(args=None):
    print("TRAINING A NEURAL NETWORK - " + NAME)
    dataset_folder = os.path.join("../dataset")
    
    epochs        = 3
    batch_size    = 50
    learning_rate = 1.0
    training_x, training_t, testing_x, testing_t = load_dataset(dataset_folder)
    nn = FCNeuralNetwork([784,30,10])
    # nn = FCNeuralNetwork([784,30,4])
    nn.train_by_SGD(training_x, training_t, epochs, batch_size, learning_rate)

    print("\nPress key to test network or ESC to exit...")
    numpy.set_printoptions(formatter={'float_kind':"{:.3f}".format})
    cmd = cv2.waitKey(0)
    while cmd != 27:
        rand_i = numpy.random.randint(0, len(testing_x))
        img,label = testing_x[rand_i], testing_t[rand_i]
        y = nn.feedforward(img)[-1]
        print("\nNN output: " + str(y.T))
        print("Expected output  : "   + str(label.T))
        print("Correctly classified: "   + str(numpy.linalg.norm(label - y) < 0.5))
        cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
        cmd = cv2.waitKey(0)
    


if __name__ == '__main__':
    main()
