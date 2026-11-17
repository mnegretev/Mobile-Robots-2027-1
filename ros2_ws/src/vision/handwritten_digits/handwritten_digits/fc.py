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
import argparse
import time

NAME = "Leonardo Santos Vicente"

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
        # TODO:
        # Calculate the output of each layer given the input x
        # Return an array y containg the output of each layer.
        # Remember that input x is considered as the layer zero
        # You can do the following steps:
        # 
        # append x to y
        # FOR i = [0,..,L-1):
        #   u = dot product (W[i], x) + B[i]
        #   x = 1.0 / (1.0 + exp(-u)) The output of the i-th layer is the input of the next one
        #   append x to y
        #
        y = [x]
        for w, b in zip(self.weights, self.biases):
            u = numpy.dot(w, x) + b
            x = 1.0 / (1.0 + numpy.exp(-u))
            y.append(x)
        return y

    def backpropagate(self, x, t):
        y = self.feedforward(x)
        nabla_b = [numpy.zeros(b.shape) for b in self.biases]
        nabla_w = [numpy.zeros(w.shape) for w in self.weights]
        # TODO:
        # Return a tuple [nabla_w, nabla_b] containing the gradient of cost function J with respect to
        # each weight and bias of all the network. The gradient is calculated assuming only one training
        # example: the input 'x' and the corresponding target 't'.
        # nabla_w and nabla_b should have the same dimensions as the corresponding
        # self.weights and self.biases
        # You can calculate the gradient following these steps:
        #
        # Calculate delta for the output layer L: delta=(y[-1]-t)*y[-1]*(1-y[-1])
        # nabla_b of output layer = delta      
        # nabla_w of output layer = delta*y[-2].T where y[-2].T is the transpose of the ouput vector of layer L-1
        # FOR all layers i=[2,L): 
        #     delta = (W[-i+1].T * delta)*y[-i]*(1 - y[-i])
        #     where 'W[-i+1].T' is the transpose of the matrix of weights of layer -i+1 and 'y[-i]' is the output of layer -i
        #     nabla_b[-i] = delta
        #     nabla_w[-i] = delta*y[-i-1].T  
        #
        # Gradientes de la capa de salida
        delta = (y[-1] - t) * y[-1] * (1 - y[-1])
        nabla_b[-1] = delta
        nabla_w[-1] = delta @ y[-2].T
        # Propagar el error hacia las capas ocultas
        for i in range(2, len(y)):
            a = y[-i]
            delta = (self.weights[-i + 1].T @ delta) * a * (1 - a)
            nabla_b[-i] = delta
            nabla_w[-i] = delta @ y[-i - 1].T
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


def load_dataset(folder, binario=True):
    print("Loading data set from " + folder)
    training_x, training_t, testing_x, testing_t = [],[],[],[]
    if binario:
        labels = [[int(bit) for bit in f"{d:04b}"] for d in range(10)]
    else:
        labels = numpy.eye(10)

    for i in range(10):
        with open(os.path.join(folder, "data" + str(i)), "rb") as archivo:
            f_data = [c / 255.0 for c in archivo.read(784000)]

        images = [
            numpy.asarray(f_data[784*j:784*(j+1)]).reshape(784, 1)
            for j in range(1000)
        ]
        label = numpy.asarray(labels[i]).reshape(-1, 1)

        mitad = len(images) // 2
        training_x += images[:mitad]
        training_t += [label] * mitad
        testing_x += images[mitad:]
        testing_t += [label] * (len(images) - mitad)
    return training_x, training_t, testing_x, testing_t

def main(args=None):
    # Configuración de argumentos por línea de comandos
    parser = argparse.ArgumentParser(description="Entrenamiento de Red Neuronal")
    parser.add_argument('--lr', type=float, default=1.0, help='Tasa de aprendizaje')
    parser.add_argument('--epochs', type=int, default=3, help='Número de épocas')
    parser.add_argument('--batch_size', type=int, default=50, help='Tamaño del lote')
    parser.add_argument('--hidden', type=int, default=30, help='Neuronas en la capa oculta')
    parser.add_argument('--num_tests', type=int, default=100, help='Número de pruebas de clasificación')
    parser.add_argument('--binario', type=int, default=1, help='1 para binario (4 bits), 0 para 10 clases')
    
    args, unknown = parser.parse_known_args()

    print("TRAINING A NEURAL NETWORK - " + NAME)
    dataset_folder = os.path.join("../dataset")
    
    # Cargar dataset usando el argumento binario
    usar_binario = bool(args.binario)
    training_x, training_t, testing_x, testing_t = load_dataset(dataset_folder, binario=usar_binario)
    
    # Inicializar la red con la capa oculta y la capa de salida dinámica
    output_neurons = 4 if usar_binario else 10
    nn = FCNeuralNetwork([784, args.hidden, output_neurons])
    
    # Registro del tiempo de entrenamiento
    start_time = time.time()
    nn.train_by_SGD(training_x, training_t, args.epochs, args.batch_size, args.lr)
    end_time = time.time()

    # Evaluación automática con el número de pruebas especificado (--num_tests)
    success = 0
    num_tests = args.num_tests
    for _ in range(num_tests):
        rand_i = numpy.random.randint(0, len(testing_x))
        img, label = testing_x[rand_i], testing_t[rand_i]
        y = nn.feedforward(img)[-1]
        if numpy.linalg.norm(label - y) < 0.5:
            success += 1
            
    print("\n\n--- RESULTADOS DE RENDIMIENTO ---")
    print(f"Tiempo de entrenamiento: {end_time - start_time:.4f} segundos")
    print(f"Porcentaje de éxitos: {(success/num_tests)*100:.2f}% (en {num_tests} pruebas)\n")

    print("\nPress key to test network manually or ESC to exit...")
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
