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
import time

NAME = "German Segovia Merlin y Juan Diego Bello Jose---2027-----1"

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
        y.append(x)
        for i in range(len(self.weights)):
            u = numpy.dot(self.weights[i], x) + self.biases[i]
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
        delta = (y[-1] - t) * y[-1] * (1 - y[-1])
        nabla_w[-1] = numpy.dot(delta, y[-2].T)
        nabla_b[-1] = delta
        
        for i in range(2, len(self.biases) + 1):
            delta = numpy.dot(self.weights[-i+1].T, delta) * y[-i] * (1 - y[-i])
            nabla_w[-i] = numpy.dot(delta, y[-i-1].T)
            nabla_b[-i] = delta
        
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
   # labels = [[1,0,0,0,0,0,0,0,0,0], [0,1,0,0,0,0,0,0,0,0], [0,0,1,0,0,0,0,0,0,0],
    #          [0,0,0,1,0,0,0,0,0,0], [0,0,0,0,1,0,0,0,0,0], [0,0,0,0,0,1,0,0,0,0],
     #         [0,0,0,0,0,0,1,0,0,0], [0,0,0,0,0,0,0,1,0,0], [0,0,0,0,0,0,0,0,1,0],
      #        [0,0,0,0,0,0,0,0,0,1]]
    labels = [[0,0,0,0], [0,0,0,1], [0,0,1,0], [0,0,1,1], [0,1,0,0],
               [0,1,0,1], [0,1,1,0], [0,1,1,1], [1,0,0,0], [1,0,0,1]]
    for i in range(10):
        f_data = [c/255.0 for c in open(os.path.join(folder, "data" + str(i)), "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784,1]) for j in range(1000)]
     #   label  = numpy.asarray(labels[i]).reshape([10,1])
        label  = numpy.asarray(labels[i]).reshape([4,1])
        training_x += images[0:len(images)//2]
        training_t += [label for j in range(len(images)//2)]
        testing_x  += images[len(images)//2:len(images)]
        testing_t  += [label for j in range(len(images)//2)]
    return training_x, training_t, testing_x, testing_t

def main(args=None):
    print("TRAINING A NEURAL NETWORK - " + NAME)
    dataset_folder = os.path.join("../dataset")
    
    epochs        = 100
    batch_size    = 100
    learning_rate = 10
    training_x, training_t, testing_x, testing_t = load_dataset(dataset_folder)
   # nn = FCNeuralNetwork([784,30,10])
    nn = FCNeuralNetwork([784,30,4])
    
    # --- INICIO CRONOMETRO ---
    start_time = time.time()
    
    nn.train_by_SGD(training_x, training_t, epochs, batch_size, learning_rate)
    
    # --- FIN CRONOMETRO ---
    end_time = time.time()
    tiempo_total = end_time - start_time
    print("\nTiempo de entrenamiento: %.2f segundos" % tiempo_total)

    # --- INICIO BLOQUE DE 100 PRUEBAS (Y GUARDAMOS LOS DATOS) ---
    print("\nRealizando 100 pruebas automaticas...")
    exitos = 0
    pruebas = 100
    
    aciertos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    intentos = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    for i in range(pruebas):
        rand_i = numpy.random.randint(0, len(testing_x))
        img = testing_x[rand_i]
        label = testing_t[rand_i]
        
        y = nn.feedforward(img)[-1]
        
        # --- EVALUACION PARA CODIGO BINARIO ---
        # Redondeamos la salida de la red a 0s y 1s
        prediccion_binaria = numpy.round(y)
        
        # Comparamos si el vector binario de la red es igual al target
        if numpy.array_equal(prediccion_binaria, label):
            exitos += 1
            # Para la tabla, usamos el numero al que corresponde el target
            # (reconstruimos el numero a partir de su binario de 4 bits)
            numero_real = int(label[0][0]*8 + label[1][0]*4 + label[2][0]*2 + label[3][0]*1)
            aciertos[numero_real] += 1
            
        # Para contar los intentos, tambien reconstruimos el numero del target
        numero_real = int(label[0][0]*8 + label[1][0]*4 + label[2][0]*2 + label[3][0]*1)
        intentos[numero_real] += 1
            
    porcentaje = (exitos / pruebas) * 100
    print("Pruebas completadas: " + str(exitos) + "/" + str(pruebas))
    print("Porcentaje de exito: %.2f%%" % porcentaje)
    # --- FIN BLOQUE DE 100 PRUEBAS ---

    # --- IMPRESION DEL RESUMEN (USANDO LOS MISMOS DATOS) ---
    print("\n==============================================")
    print("       RESUMEN DE LAS 100 PRUEBAS")
    print("==============================================")
    print("Numero | Aciertos | Intentos | % Exito")
    print("----------------------------------------------")
    for numero in range(10):
        if intentos[numero] > 0:
            porc = (aciertos[numero] / intentos[numero]) * 100
        else:
            porc = 0.0
        print("   %d   |    %d     |    %d     |  %.1f%%" % (numero, aciertos[numero], intentos[numero], porc))
    print("----------------------------------------------")
    print("TOTAL  |    %d    |   %d    |  %.1f%%" % (exitos, pruebas, porcentaje))
    print("==============================================\n")
    # --- FIN DEL RESUMEN ---

    # --- PRUEBA VISUAL CON VENTANITA (OPCIONAL) ---
    print("Presiona una tecla para ver imagenes (ESC para salir)...")
    numpy.set_printoptions(formatter={'float_kind':"{:.3f}".format})
    
    while True:
        rand_i = numpy.random.randint(0, len(testing_x))
        img, label = testing_x[rand_i], testing_t[rand_i]
        y = nn.feedforward(img)[-1]
        
        prediccion = numpy.argmax(y)
        objetivo = numpy.argmax(label)
        confianza = numpy.max(y)
        
        if prediccion == objetivo and confianza >= 0.9:
            print("\nNN output: " + str(y.T))
            print("Expected output  : " + str(label.T))
            print("Correctly classified: True (Confianza: " + str(round(confianza, 2)) + ")")
            cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
            break 
            
    if cv2.waitKey(0) == 27:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
