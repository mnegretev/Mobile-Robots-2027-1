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
import csv

NAME = "Solorio Gonzalez Aldo Bruno"

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
        
        # Error de la capa de salida.
        delta = (y[-1] - t) * y[-1] * (1.0 - y[-1])

        # Gradientes de los sesgos y pesos de la capa de salida.
        nabla_b[-1] = delta
        nabla_w[-1] = numpy.dot(delta, y[-2].T)

        # Propagación del error hacia las capas ocultas.
        for i in range(2, len(self.weights) + 1):
            delta = (
                numpy.dot(self.weights[-i + 1].T, delta)
                * y[-i]
                * (1.0 - y[-i])
            )

            nabla_b[-i] = delta
            nabla_w[-i] = numpy.dot(delta, y[-i - 1].T)

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
    labels = [
        [0, 0, 0, 0],  # 0
        [0, 0, 0, 1],  # 1
        [0, 0, 1, 0],  # 2
        [0, 0, 1, 1],  # 3
        [0, 1, 0, 0],  # 4
        [0, 1, 0, 1],  # 5
        [0, 1, 1, 0],  # 6
        [0, 1, 1, 1],  # 7
        [1, 0, 0, 0],  # 8
        [1, 0, 0, 1],  # 9
    ]
    # labels = [[0,0,0,0], [0,0,0,1], [0,0,1,0], [0,0,1,1], [0,1,0,0],
    #           [0,1,0,1], [0,1,1,0], [0,1,1,1], [1,0,0,0], [1,0,0,1]]
    for i in range(10):
        f_data = [c/255.0 for c in open(os.path.join(folder, "data" + str(i)), "rb").read(784000)]
        images = [numpy.asarray(f_data[784*j:784*(j+1)]).reshape([784,1]) for j in range(1000)]
        label = numpy.asarray(labels[i]).reshape([4,1])        # label  = numpy.asarray(labels[i]).reshape([4,1])
        training_x += images[0:len(images)//2]
        training_t += [label for j in range(len(images)//2)]
        testing_x  += images[len(images)//2:len(images)]
        testing_t  += [label for j in range(len(images)//2)]
    return training_x, training_t, testing_x, testing_t



# def main(args=None):
#     print("TRAINING A NEURAL NETWORK - " + NAME)
#     dataset_folder = os.path.join("../dataset")
    
#     epochs        = 3
#     batch_size    = 50
#     learning_rate = 1.0
#     training_x, training_t, testing_x, testing_t = load_dataset(dataset_folder)
#     nn = FCNeuralNetwork([784,30,10])
#     # nn = FCNeuralNetwork([784,30,4])
#     nn.train_by_SGD(training_x, training_t, epochs, batch_size, learning_rate)

#     print("\nPress key to test network or ESC to exit...")
#     numpy.set_printoptions(formatter={'float_kind':"{:.3f}".format})
#     # cmd = cv2.waitKey(0)
    
#     cmd = 0
    
#     while cmd != 27:
#         rand_i = numpy.random.randint(0, len(testing_x))
#         img,label = testing_x[rand_i], testing_t[rand_i]
#         y = nn.feedforward(img)[-1]
#         print("\nNN output: " + str(y.T))
#         print("Expected output  : "   + str(label.T))
#         print("Correctly classified: "   + str(numpy.linalg.norm(label - y) < 0.5))
#         cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
#         cmd = cv2.waitKey(0)


def decodificar_binario(salida):
    bits = (numpy.asarray(salida).reshape(-1) >= 0.5).astype(int)
    pesos = numpy.array([8, 4, 2, 1])
    return int(numpy.dot(bits, pesos))


def main(args=None):
    print("TRAINING A NEURAL NETWORK - " + NAME)

    dataset_folder = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "../dataset"
    )

    training_x, training_t, testing_x, testing_t = load_dataset(
        dataset_folder
    )

    # Configuración de referencia:
    # tasa = 1.0, épocas = 10, tamaño del lote = 30.
    #
    # Se modifica un parámetro a la vez.
    # La referencia se ejecuta una sola vez y se comparte
    # entre las tres comparaciones.
    configuraciones = [
        ("BASE",     1.0,  10,  30),
        ("TASA_05",  0.5,  10,  30),
        ("TASA_3",   3.0,  10,  30),
        ("TASA_10", 10.0,  10,  30),
        ("EPOCAS_3", 1.0,   3,  30),
        ("EPOCAS_50", 1.0, 50,  30),
        ("EPOCAS_100", 1.0, 100, 30),
        ("LOTE_5",   1.0,  10,   5),
        ("LOTE_10",  1.0,  10,  10),
        ("LOTE_100", 1.0,  10, 100),
    ]

    semilla = 42

    # Elegir 10 imágenes por dígito: 100 en total.
    # Se usan las mismas imágenes en todas las configuraciones.
    # La selección pertenece exclusivamente al conjunto de prueba.
    rng = numpy.random.default_rng(semilla)
    etiquetas = numpy.array([
        decodificar_binario(t) for t in testing_t
    ])

    indices_prueba = []

    for digito in range(10):
        candidatos = numpy.flatnonzero(etiquetas == digito)
        seleccion = rng.choice(candidatos, size=10, replace=False)
        indices_prueba.extend(seleccion.tolist())

    # Guardar los CSV junto al script.
    carpeta = os.path.dirname(os.path.abspath(__file__))
    marca = time.strftime("%Y%m%d_%H%M%S")

    archivo_csv = os.path.join(
        carpeta, f"resultados_binario_{marca}.csv"        
    )
    archivo_detalle = os.path.join(
        carpeta, f"clasificaciones_binario_{marca}.csv"
    )

    with open(
        archivo_csv, "x", newline="", encoding="utf-8"
    ) as archivo, open(
        archivo_detalle, "x", newline="", encoding="utf-8"
    ) as detalle:

        escritor = csv.writer(archivo)
        escritor_detalle = csv.writer(detalle)

        escritor.writerow([
            "configuracion",
            "arquitectura",
            "semilla",
            "tasa_aprendizaje",
            "epocas",
            "tamano_lote",
            "imagenes_entrenamiento",
            "imagenes_prueba",
            "tiempo_entrenamiento_s",
            "aciertos_criterio_original",
            "porcentaje_criterio_original",
            "aciertos_por_digito",
            "porcentaje_por_digito"
        ])

        escritor_detalle.writerow([
            "configuracion",
            "indice_prueba",
            "digito_real",
            "digito_predicho",
            "distancia_salida_etiqueta",
            "acierto_criterio_original",
            "acierto_por_digito"
        ])

        for numero, configuracion in enumerate(
            configuraciones, start=1
        ):
            nombre, tasa, epocas, lote = configuracion

            print("\n" + "=" * 60)
            print(
                f"EXPERIMENTO {numero}/{len(configuraciones)}: "
                f"{nombre}"
            )
            print(
                f"Tasa: {tasa} | Épocas: {epocas} | Lote: {lote}"
            )

            # Cada experimento comienza con una red nueva.
            # Se repite la semilla para controlar la inicialización
            # y el orden aleatorio de los datos.
            numpy.random.seed(semilla)
            random.seed(semilla)

            nn = FCNeuralNetwork([784, 30, 4])

            inicio = time.perf_counter()

            nn.train_by_SGD(
                training_x,
                training_t,
                epocas,
                lote,
                tasa
            )

            tiempo_entrenamiento = time.perf_counter() - inicio

            aciertos_original = 0
            aciertos_digito = 0

            for indice in indices_prueba:
                imagen = testing_x[indice]
                etiqueta = testing_t[indice]

                salida = nn.feedforward(imagen)[-1]

                distancia = float(
                    numpy.linalg.norm(etiqueta - salida)
                )

                # Criterio que viene en el código del profesor.
                correcto_original = distancia < 0.5

                # Medida adicional: identificar el dígito.
                real = decodificar_binario(etiqueta)
                predicho = decodificar_binario(salida)
                correcto_digito = real == predicho

                aciertos_original += int(correcto_original)
                aciertos_digito += int(correcto_digito)

                escritor_detalle.writerow([
                    nombre,
                    indice,
                    real,
                    predicho,
                    distancia,
                    int(correcto_original),
                    int(correcto_digito)
                ])

            total = len(indices_prueba)

            porcentaje_original = (
                100.0 * aciertos_original / total
            )
            porcentaje_digito = (
                100.0 * aciertos_digito / total
            )

            escritor.writerow([
                nombre,
                "784-30-4",
                semilla,
                tasa,
                epocas,
                lote,
                len(training_x),
                total,
                tiempo_entrenamiento,
                aciertos_original,
                porcentaje_original,
                aciertos_digito,
                porcentaje_digito
            ])

            # Conservar cada experimento terminado en disco.
            archivo.flush()
            detalle.flush()

            print(
                f"\nTiempo de entrenamiento: "
                f"{tiempo_entrenamiento:.3f} s"
            )
            print(
                f"Aciertos, criterio original: "
                f"{aciertos_original}/{total} "
                f"({porcentaje_original:.2f} %)"
            )
            print(
                f"Aciertos por dígito: "
                f"{aciertos_digito}/{total} "
                f"({porcentaje_digito:.2f} %)"
            )

    print("\nEXPERIMENTOS TERMINADOS")
    print("Resultados:", archivo_csv)
    print("Detalle:", archivo_detalle)




if __name__ == '__main__':
    main()
