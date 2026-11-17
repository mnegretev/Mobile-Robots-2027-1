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

NAME = "Omar Rosario Carrasco"

# Los imports adicionales van aqui abajo para que NAME se quede en la linea 15.
import argparse
import contextlib
import csv
import time

try:
    import sympy
except ImportError:     # El programa debe correr aunque SymPy no este instalado
    sympy = None

#
# DERIVADAS CON SYMPY
# Backpropagation necesita dos derivadas: la de la funcion de costo respecto a la salida de la
# red y la de la funcion de activacion. En lugar de escribirlas a mano, se derivan de forma
# simbolica con SymPy y se convierten una sola vez en funciones de NumPy con lambdify, asi que
# el entrenamiento cuesta lo mismo que con las formulas escritas a mano.
#
def build_derivatives():
    if sympy is None:
        print("SymPy is not installed: using the hand-written derivatives")
        activation   = lambda u: 1.0/(1.0 + numpy.exp(-u))
        d_activation = lambda y: y*(1.0 - y)
        d_cost       = lambda y, t: y - t
        return activation, d_activation, d_cost, None
    u, y, t = sympy.symbols('u y t', real=True)
    sigma = 1/(1 + sympy.exp(-u))               # Funcion de activacion: y = sigma(u)
    cost  = (y - t)**2/2                        # Costo de una salida para un ejemplo
    dsigma_du = sympy.diff(sigma, u)            # exp(-u)/(1 + exp(-u))**2
    # feedforward guarda las salidas 'y' de cada capa, no las 'u'. Para escribir la derivada en
    # terminos de 'y' se sustituye la inversa de la sigmoide, u = ln(y/(1-y)), y se simplifica.
    logit = sympy.log(y/(1 - y))
    assert sympy.simplify(sigma.subs(u, logit) - y) == 0
    dsigma_dy = sympy.factor(sympy.simplify(dsigma_du.subs(u, logit)))      # y*(1 - y)
    dcost_dy  = sympy.diff(cost, y)                                         # y - t
    activation   = sympy.lambdify(u, sigma, 'numpy')
    d_activation = sympy.lambdify(y, dsigma_dy, 'numpy')
    d_cost       = sympy.lambdify((y, t), dcost_dy, 'numpy')
    symbolic = {'u':u, 'y':y, 't':t, 'sigma':sigma, 'dsigma_du':dsigma_du, 'dsigma_dy':dsigma_dy, 'dcost_dy':dcost_dy}
    return activation, d_activation, d_cost, symbolic

ACTIVATION, D_ACTIVATION, D_COST, SYMBOLIC = build_derivatives()

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
        y.append(x)                                     # La entrada es la capa cero
        for i in range(len(self.weights)):
            u = numpy.dot(self.weights[i], x) + self.biases[i]
            x = ACTIVATION(u)                           # 1/(1 + exp(-u)), generada con SymPy
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
        # D_COST(y,t) = y - t es dJ/dy y D_ACTIVATION(y) = y*(1 - y) es dy/du (ver build_derivatives).
        # numpy.dot es el producto matricial; el '*' entre arreglos es elemento a elemento.
        delta = D_COST(y[-1], t)*D_ACTIVATION(y[-1])
        nabla_b[-1] = delta
        nabla_w[-1] = numpy.dot(delta, y[-2].T)
        for i in range(2, len(y)):                      # len(y) = L porque 'y' incluye la capa cero
            delta = numpy.dot(self.weights[-i+1].T, delta)*D_ACTIVATION(y[-i])
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


def load_dataset(folder, binary_labels=False):
    print("Loading data set from " + folder)
    training_x, training_t, testing_x, testing_t = [],[],[],[]
    labels = [[1,0,0,0,0,0,0,0,0,0], [0,1,0,0,0,0,0,0,0,0], [0,0,1,0,0,0,0,0,0,0],
              [0,0,0,1,0,0,0,0,0,0], [0,0,0,0,1,0,0,0,0,0], [0,0,0,0,0,1,0,0,0,0],
              [0,0,0,0,0,0,1,0,0,0], [0,0,0,0,0,0,0,1,0,0], [0,0,0,0,0,0,0,0,1,0],
              [0,0,0,0,0,0,0,0,0,1]]
    if binary_labels:
        # Etiquetas en codigo binario: cada digito se escribe con 4 bits (el 5 es 0101)
        labels = [[0,0,0,0], [0,0,0,1], [0,0,1,0], [0,0,1,1], [0,1,0,0],
                  [0,1,0,1], [0,1,1,0], [0,1,1,1], [1,0,0,0], [1,0,0,1]]
    for i in range(10):
        f_data = [c/255.0 for c in open(os.path.join(folder, "data" + str(i)), "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784,1]) for j in range(1000)]
        label  = numpy.asarray(labels[i]).reshape([len(labels[i]),1])      # 10x1 (one-hot) o 4x1 (binario)
        training_x += images[0:len(images)//2]
        training_t += [label for j in range(len(images)//2)]
        testing_x  += images[len(images)//2:len(images)]
        testing_t  += [label for j in range(len(images)//2)]
    return training_x, training_t, testing_x, testing_t

#
# PRUEBAS DE DESEMPENO (puntos 4 y 5 de la practica)
#
LEARNING_RATES = [0.5, 1.0, 3.0, 10.0]
EPOCHS         = [3, 10, 50, 100]
BATCH_SIZES    = [5, 10, 30, 100]
CSV_HEADER     = ["etiquetas", "arquitectura", "tasa_aprendizaje", "epocas", "lote", "semilla",
                  "tiempo_entrenamiento_s", "aciertos_100_pct", "aciertos_prueba_pct", "digito_prueba_pct",
                  "costo_prueba", "aciertos_entrenamiento_pct", "digito_entrenamiento_pct", "costo_entrenamiento"]

def test_network(nn, data_x, data_t, binary_labels, indices=None):
    #
    # Prueba la red con los ejemplos indicados (todos, si no se indican) y devuelve:
    #   - Porcentaje de aciertos con el criterio de la plantilla: ||t - y|| < 0.5
    #   - Porcentaje de digitos bien decodificados: la neurona mas activa es la correcta (one-hot)
    #     o los 4 bits redondeados coinciden con la etiqueta (binario)
    #   - Costo promedio J = 1/(2N) * suma ||y - t||^2
    #
    indices = range(len(data_x)) if indices is None else indices
    hits, decoded, cost = 0, 0, 0.0
    for i in indices:
        t = data_t[i]
        y = nn.feedforward(data_x[i])[-1]
        error = numpy.linalg.norm(t - y)
        cost += 0.5*error**2
        if error < 0.5:
            hits += 1
        if binary_labels:
            decoded += int(numpy.array_equal(numpy.round(y), t))
        else:
            decoded += int(numpy.argmax(y) == numpy.argmax(t))
    return 100.0*hits/len(indices), 100.0*decoded/len(indices), cost/len(indices)

def run_experiments(options, dataset_folder, start_time):
    #
    # Entrena una red nueva por cada combinacion de tasa de aprendizaje, numero de epocas y tamano
    # de lote (4 x 4 x 4 = 64 combinaciones) y guarda un renglon por combinacion en un archivo CSV.
    # Si el CSV ya tiene renglones, esas combinaciones se saltan (sirve para reanudar).
    #
    training_x, training_t, testing_x, testing_t = load_dataset(dataset_folder, options.binary)
    layers       = [784] + options.hidden + [4 if options.binary else 10]
    architecture = "-".join([str(n) for n in layers])
    label_type   = "binario" if options.binary else "one-hot"
    seed         = 2027 if options.seed is None else options.seed
    done, seconds_per_epoch = set(), 0.0       # seconds_per_epoch: el mayor tiempo por epoca observado
    if os.path.exists(options.csv):
        for row in csv.DictReader(open(options.csv, newline="")):
            done.add((float(row["tasa_aprendizaje"]), int(row["epocas"]), int(row["lote"])))
            seconds_per_epoch = max(seconds_per_epoch, float(row["tiempo_entrenamiento_s"])/int(row["epocas"]))
    else:
        with open(options.csv, "w", newline="") as f:
            csv.writer(f).writerow(CSV_HEADER)
    print("Experiments: labels=%s architecture=%s seed=%d csv=%s (%d already done)" %
          (label_type, architecture, seed, options.csv, len(done)))
    pending = 0
    for learning_rate in LEARNING_RATES:
        for epochs in EPOCHS:
            for batch_size in BATCH_SIZES:
                if (learning_rate, epochs, batch_size) in done:
                    continue
                if options.max_seconds > 0:     # Limite de tiempo opcional para correr el barrido por partes
                    remaining = options.max_seconds - (time.perf_counter() - start_time)
                    if 1.2*(seconds_per_epoch if seconds_per_epoch > 0 else 1.0)*epochs + 3.0 > remaining:
                        pending += 1
                        continue
                # Misma semilla en todas las combinaciones: todas parten de los mismos pesos iniciales
                numpy.random.seed(seed)
                random.seed(seed)
                nn = FCNeuralNetwork(layers)
                # El avance que imprime train_by_SGD se manda a /dev/null para que el tiempo medido
                # sea el del entrenamiento y no el de escribir en la terminal.
                with open(os.devnull, "w") as devnull, contextlib.redirect_stdout(devnull):
                    t0 = time.perf_counter()
                    nn.train_by_SGD(training_x, training_t, epochs, batch_size, learning_rate)
                    training_time = time.perf_counter() - t0
                seconds_per_epoch = max(seconds_per_epoch, training_time/epochs)
                # Pruebas de clasificacion: 100 imagenes de prueba al azar, como pide la practica,
                # y ademas todas las de prueba (5000) y todas las de entrenamiento (5000).
                sample = numpy.random.randint(0, len(testing_x), options.tests)
                hits_sample, _, _                       = test_network(nn, testing_x,  testing_t,  options.binary, sample)
                hits_test,  decoded_test,  cost_test    = test_network(nn, testing_x,  testing_t,  options.binary)
                hits_train, decoded_train, cost_train   = test_network(nn, training_x, training_t, options.binary)
                with open(options.csv, "a", newline="") as f:
                    csv.writer(f).writerow([label_type, architecture, learning_rate, epochs, batch_size, seed,
                                            "%.3f" % training_time, "%.1f" % hits_sample, "%.2f" % hits_test,
                                            "%.2f" % decoded_test, "%.5f" % cost_test, "%.2f" % hits_train,
                                            "%.2f" % decoded_train, "%.5f" % cost_train])
                done.add((learning_rate, epochs, batch_size))
                print("eta=%4.1f epochs=%3d batch=%3d | time=%7.2f s | hits: %5.1f %% (100 tests) %6.2f %% (test set) %6.2f %% (training set)" %
                      (learning_rate, epochs, batch_size, training_time, hits_sample, hits_test, hits_train))
                sys.stdout.flush()
    print("Done: %d of 64 combinations in %s. Pending: %d" % (len(done), options.csv, pending))

def check_backpropagation_with_sympy(layers=[4,3,2], seed=7):
    #
    # Comprobacion de backpropagate: en una red pequena se arma con SymPy la funcion de costo J
    # completa, con todos los pesos y bias como simbolos, se deriva respecto a cada uno y el
    # resultado se compara con el gradiente que devuelve backpropagate.
    #
    if sympy is None:
        print("SymPy is required for this check (pip install sympy)")
        return False
    numpy.random.seed(seed)
    nn = FCNeuralNetwork(layers)
    x  = numpy.random.rand(layers[0], 1)
    t  = numpy.round(numpy.random.rand(layers[-1], 1))
    u, sigma = SYMBOLIC['u'], SYMBOLIC['sigma']
    W = [sympy.Matrix(w.shape[0], w.shape[1], lambda r,c,k=k: sympy.Symbol("w%d_%d_%d" % (k+1,r+1,c+1)))
         for k,w in enumerate(nn.weights)]
    B = [sympy.Matrix(b.shape[0], 1, lambda r,c,k=k: sympy.Symbol("b%d_%d" % (k+1,r+1)))
         for k,b in enumerate(nn.biases)]
    Y = sympy.Matrix(x.tolist())                # Salida de la capa cero
    for Wk, Bk in zip(W, B):
        Y = (Wk*Y + Bk).applyfunc(lambda e: sigma.subs(u, e))
    J = sum([(Y[k] - t[k,0])**2 for k in range(layers[-1])])/2
    values = {}
    for k in range(len(W)):
        values.update({W[k][r,c]: nn.weights[k][r,c] for r in range(W[k].rows) for c in range(W[k].cols)})
        values.update({B[k][r,0]: nn.biases[k][r,0] for r in range(B[k].rows)})
    nabla_w, nabla_b = nn.backpropagate(x, t)
    max_difference = 0.0
    for k in range(len(W)):
        sym_w = numpy.asarray([[float(sympy.diff(J, W[k][r,c]).evalf(subs=values)) for c in range(W[k].cols)]
                               for r in range(W[k].rows)])
        sym_b = numpy.asarray([[float(sympy.diff(J, B[k][r,0]).evalf(subs=values))] for r in range(B[k].rows)])
        max_difference = max(max_difference, numpy.abs(sym_w - nabla_w[k]).max(), numpy.abs(sym_b - nabla_b[k]).max())
    print("Network %s: %d partial derivatives of J obtained with SymPy" % (str(layers), len(values)))
    print("Largest difference against backpropagate: %.3e" % max_difference)
    print("CHECK PASSED" if max_difference < 1e-9 else "CHECK FAILED")
    return max_difference < 1e-9

def parse_options(args=None):
    parser = argparse.ArgumentParser(description="Fully connected neural network for handwritten digit recognition")
    parser.add_argument("--binary", action="store_true", help="labels in binary code (4 outputs) instead of one-hot (10 outputs)")
    parser.add_argument("--hidden", default="30", help="neurons in the hidden layers, e.g. 30 or 30,20 (default: 30)")
    # Valores por omision: la combinacion con mejor relacion entre aciertos y tiempo de entrenamiento
    # en los experimentos del reporte. La plantilla traia 3 epocas, lotes de 50 y tasa de 1.0.
    parser.add_argument("--epochs", type=int, default=10)
    parser.add_argument("--batch-size", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=3.0)
    parser.add_argument("--dataset", default="../dataset", help="folder with the files data0 ... data9")
    parser.add_argument("--seed", type=int, default=None, help="seed for the initial weights and the shuffling")
    parser.add_argument("--experiments", action="store_true", help="run the 64 combinations of learning rate, epochs and batch size")
    parser.add_argument("--csv", default="experiments.csv", help="file where the experiments are saved")
    parser.add_argument("--tests", type=int, default=100, help="random classification tests per combination")
    parser.add_argument("--max-seconds", type=float, default=0, help="stop starting new combinations after this time (0 = no limit)")
    parser.add_argument("--check", action="store_true", help="compare backpropagate against the gradient derived with SymPy")
    options, _ = parser.parse_known_args(args)
    options.hidden = [int(n) for n in options.hidden.split(",") if n.strip() != ""]
    return options

def main(args=None):
    start_time = time.perf_counter()
    print("TRAINING A NEURAL NETWORK - " + NAME)
    options = parse_options(args)
    numpy.seterr(over="ignore")     # Con u muy negativa exp(-u) se desborda a inf y 1/(1+inf) = 0, que es el limite correcto
    if SYMBOLIC is not None:
        print("Derivatives obtained with SymPy: dy/du = " + str(SYMBOLIC['dsigma_dy']) + ", dJ/dy = " + str(SYMBOLIC['dcost_dy']))
    if options.check:
        check_backpropagation_with_sympy()
        return
    dataset_folder = os.path.join(options.dataset)
    if options.experiments:
        run_experiments(options, dataset_folder, start_time)
        return
    
    epochs        = options.epochs
    batch_size    = options.batch_size
    learning_rate = options.learning_rate
    if options.seed is not None:
        numpy.random.seed(options.seed)
        random.seed(options.seed)
    training_x, training_t, testing_x, testing_t = load_dataset(dataset_folder, options.binary)
    # La capa de salida debe ser congruente con las etiquetas: 10 neuronas (one-hot) o 4 (binario)
    nn = FCNeuralNetwork([784] + options.hidden + [4 if options.binary else 10])
    t0 = time.perf_counter()
    nn.train_by_SGD(training_x, training_t, epochs, batch_size, learning_rate)
    print("Training time: %.2f s" % (time.perf_counter() - t0))

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
